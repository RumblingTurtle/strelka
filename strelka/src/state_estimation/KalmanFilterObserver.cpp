#include <strelka/state_estimation/KalmanFilterObserver.hpp>

namespace strelka {

namespace state_estimation {
const int KalmanFilterObserver::STATE_DIM;
const int KalmanFilterObserver::SENSOR_DIM;

KalmanFilterObserver::KalmanFilterObserver(
    KalmanFilterObserverParams &parameters)
    : initialized(false), parameters(parameters) {
  initialize();
}

KalmanFilterObserver::KalmanFilterObserver()
    : initialized(false), parameters() {}

void KalmanFilterObserver::setParameters(
    KalmanFilterObserverParams &parameters) {
  this->parameters = parameters;
  initialize();
}

void KalmanFilterObserver::initialize() {
  _xhat.setZero();
  _ps.setZero();
  _vs.setZero();

  _A.setZero();
  _A.block<3, 3>(0, 0) = Eigen::Matrix<float, 3, 3>::Identity();
  _A.block<3, 3>(0, 3) = parameters.dt * Eigen::Matrix<float, 3, 3>::Identity();
  _A.block<3, 3>(3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
  _A.block<12, 12>(6, 6) = Eigen::Matrix<float, 12, 12>::Identity();
  _B.setZero();
  _B.block<3, 3>(3, 0) = parameters.dt * Eigen::Matrix<float, 3, 3>::Identity();
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<float, 3, 3>::Identity(),
      Eigen::Matrix<float, 3, 3>::Zero();
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<float, 3, 3>::Zero(),
      Eigen::Matrix<float, 3, 3>::Identity();
  _C.setZero();
  _C.block<3, 6>(0, 0) = C1;
  _C.block<3, 6>(3, 0) = C1;
  _C.block<3, 6>(6, 0) = C1;
  _C.block<3, 6>(9, 0) = C1;
  _C.block<12, 12>(0, 6) = float(-1) * Eigen::Matrix<float, 12, 12>::Identity();
  _C.block<3, 6>(12, 0) = C2;
  _C.block<3, 6>(15, 0) = C2;
  _C.block<3, 6>(STATE_DIM, 0) = C2;
  _C.block<3, 6>(21, 0) = C2;
  _C(27, 17) = float(1);
  _C(26, 14) = float(1);
  _C(25, 11) = float(1);
  _C(24, 8) = float(1);

  _P.setIdentity();
  _P = float(0.001) * _P;
  _Q0.setIdentity();
  _Q0.block<3, 3>(0, 0) =
      parameters.dt * Eigen::Matrix<float, 3, 3>::Identity();
  _Q0.block<3, 3>(3, 3) =
      parameters.dt * Eigen::Matrix<float, 3, 3>::Identity();
  _Q0.block<12, 12>(6, 6) =
      parameters.dt * Eigen::Matrix<float, 12, 12>::Identity();
  _R0.setIdentity();

  initialized = true;
}

void KalmanFilterObserver::update(robots::Robot &robot,
                                  bool useExternalOdometry,
                                  Vec3<float> externalOdometryPosition,
                                  Vec4<float> contactHeights) {
  if (!initialized) {
    throw UninitializedKalmanFilter();
  }

  Eigen::Matrix<float, STATE_DIM, STATE_DIM> Q =
      Eigen::Matrix<float, STATE_DIM, STATE_DIM>::Identity();

  Q.block<3, 3>(0, 0) =
      _Q0.block<3, 3>(0, 0) * parameters.imuPositionProcessNoise;
  Q.block<3, 3>(3, 3) =
      _Q0.block<3, 3>(3, 3) * parameters.imuVelocityProcessNoise;
  Q.block<12, 12>(6, 6) =
      _Q0.block<12, 12>(6, 6) * parameters.footPositionProcessNoise;

  Eigen::Matrix<float, SENSOR_DIM, SENSOR_DIM> R =
      Eigen::Matrix<float, SENSOR_DIM, SENSOR_DIM>::Identity();
  R.block<12, 12>(0, 0) =
      _R0.block<12, 12>(0, 0) * parameters.footPositionSensorNoise;
  R.block<12, 12>(12, 12) =
      _R0.block<12, 12>(12, 12) * parameters.footVelocitySensorNoise;
  R.block<4, 4>(24, 24) =
      _R0.block<4, 4>(24, 24) * parameters.contactHeightSensorNoise;

  R(28, 28) = _R0(28, 28) * parameters.externalOdometryNoisePosition[0];
  R(29, 29) = _R0(29, 29) * parameters.externalOdometryNoisePosition[1];
  R(30, 30) = _R0(30, 30) * parameters.externalOdometryNoisePosition[2];

  if (useExternalOdometry) {
    // Loam state to sensor
    _C(28, 0) = float(1);
    _C(29, 1) = float(1);
    _C(30, 2) = float(0);
  } else {
    _C(28, 0) = float(0);
    _C(29, 1) = float(0);
    _C(30, 2) = float(0);
  }

  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;

  Eigen::Matrix<float, 4, 1> pzs = Eigen::Matrix<float, 4, 1>::Zero();
  Vec3<float> p0, v0;
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];

  Eigen::Matrix<float, 4, 1> trusts = Eigen::Matrix<float, 4, 1>::Zero();
  float h_i = 0.0f;
  FOR_EACH_LEG {
    int i1 = 3 * LEG_ID;
    Vec3<float> p_rel = robot.footPositionTrunkFrame(LEG_ID);
    Vec3<float> dp_rel = robot.footVelocityTrunkFrame(LEG_ID);
    Vec3<float> p_f = robot.rotateBodyToWorldFrame(p_rel);
    Vec3<float> dp_f = robot.rotateBodyToWorldFrame(
        robot.gyroscopeBodyFrame().cross(p_rel) + dp_rel);
    float contactHeight = contactHeights(LEG_ID);
    qindex = 6 + i1;
    rindex1 = i1;
    rindex2 = 12 + i1;
    rindex3 = 24 + LEG_ID;

    float trust = robot.footContact(LEG_ID);

    float high_suspect_number(100);

    Q.block<3, 3>(qindex, qindex) =
        (float(1) + (float(1) - trust) * high_suspect_number) *
        Q.block<3, 3>(qindex, qindex);
    R.block<3, 3>(rindex1, rindex1) = R.block<3, 3>(rindex1, rindex1);
    R.block<3, 3>(rindex2, rindex2) =
        (float(1) + (float(1) - trust) * high_suspect_number) *
        R.block<3, 3>(rindex2, rindex2);
    R(rindex3, rindex3) =
        (float(1) + (float(1) - trust) * high_suspect_number) *
        R(rindex3, rindex3);

    trusts(LEG_ID) = trust;

    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);

    pzs(LEG_ID) = (1.0f - trust) * (p0(2) + p_f(2)) + trust * contactHeight;
  }

  Eigen::Matrix<float, SENSOR_DIM, 1> y;
  y << _ps, _vs, pzs, externalOdometryPosition;
  _xhat = _A * _xhat + _B * robot.accelerometerWorldFrame();

  Eigen::Matrix<float, STATE_DIM, STATE_DIM> At = _A.transpose();
  Eigen::Matrix<float, STATE_DIM, STATE_DIM> Pm = _A * _P * At + Q;
  Eigen::Matrix<float, STATE_DIM, SENSOR_DIM> Ct = _C.transpose();
  Eigen::Matrix<float, SENSOR_DIM, 1> yModel = _C * _xhat;
  Eigen::Matrix<float, SENSOR_DIM, 1> ey = y - yModel;
  Eigen::Matrix<float, SENSOR_DIM, SENSOR_DIM> S = _C * Pm * Ct + R;

  Eigen::Matrix<float, SENSOR_DIM, 1> S_ey = S.lu().solve(ey);
  _xhat += Pm * Ct * S_ey;

  Eigen::Matrix<float, SENSOR_DIM, STATE_DIM> S_C = S.lu().solve(_C);
  _P =
      (Eigen::Matrix<float, STATE_DIM, STATE_DIM>::Identity() - Pm * Ct * S_C) *
      Pm;

  Eigen::Matrix<float, STATE_DIM, STATE_DIM> Pt = _P.transpose();
  _P = (_P + Pt) / float(2);

  _position = _xhat.block<3, 1>(0, 0);
  _velocityWorld = _xhat.block<3, 1>(3, 0);
  _velocityBody = robot.rotateWorldToBodyFrame(_xhat.block<3, 1>(3, 0));
  _footPositionsWorld = _xhat.block<12, 1>(6, 0);
  _positionCovariance = _P.block<3, 3>(0, 0);
}

Vec3<float> KalmanFilterObserver::position() const { return _position; }

Vec3<float> KalmanFilterObserver::velocityBody() const {
  return _velocityBody;
};

Vec3<float> KalmanFilterObserver::velocityWorld() const {
  return _velocityWorld;
};

Vec12<float> KalmanFilterObserver::footPositionsWorld() const {
  return _footPositionsWorld;
};

Eigen::Matrix3f KalmanFilterObserver::positionCovariance() const {
  return _positionCovariance;
};

void KalmanFilterObserver::reset() { _xhat.setZero(); }
} // namespace state_estimation
} // namespace strelka
