#include <state_estimation/KalmanFilterObserver.hpp>

namespace strelka {

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
  _A.block(0, 0, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
  _A.block(0, 3, 3, 3) = parameters.dt * Eigen::Matrix<float, 3, 3>::Identity();
  _A.block(3, 3, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
  _A.block(6, 6, 12, 12) = Eigen::Matrix<float, 12, 12>::Identity();
  _B.setZero();
  _B.block(3, 0, 3, 3) = parameters.dt * Eigen::Matrix<float, 3, 3>::Identity();
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<float, 3, 3>::Identity(),
      Eigen::Matrix<float, 3, 3>::Zero();
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<float, 3, 3>::Zero(),
      Eigen::Matrix<float, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  _C.block(6, 0, 3, 6) = C1;
  _C.block(9, 0, 3, 6) = C1;
  _C.block(0, 6, 12, 12) = float(-1) * Eigen::Matrix<float, 12, 12>::Identity();
  _C.block(12, 0, 3, 6) = C2;
  _C.block(15, 0, 3, 6) = C2;
  _C.block(STATE_DIM, 0, 3, 6) = C2;
  _C.block(21, 0, 3, 6) = C2;
  _C(27, 17) = float(1);
  _C(26, 14) = float(1);
  _C(25, 11) = float(1);
  _C(24, 8) = float(1);

  _P.setIdentity();
  _P = float(0.001) * _P;
  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) =
      parameters.dt * Eigen::Matrix<float, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3) =
      parameters.dt * Eigen::Matrix<float, 3, 3>::Identity();
  _Q0.block(6, 6, 12, 12) =
      parameters.dt * Eigen::Matrix<float, 12, 12>::Identity();
  _R0.setIdentity();

  initialized = true;
}

void KalmanFilterObserver::update(KalmanFilterObserverInput &inputs,
                                  KalmanFilterObserverOutput &output) {
  if (!initialized) {
    throw UninitializedKalmanFilter();
  }
  Eigen::Matrix<float, STATE_DIM, STATE_DIM> Q =
      Eigen::Matrix<float, STATE_DIM, STATE_DIM>::Identity();

  Q.block(0, 0, 3, 3) =
      _Q0.block(0, 0, 3, 3) * parameters.imuPositionProcessNoise;
  Q.block(3, 3, 3, 3) =
      _Q0.block(3, 3, 3, 3) * parameters.imuVelocityProcessNoise;
  Q.block(6, 6, 12, 12) =
      _Q0.block(6, 6, 12, 12) * parameters.footPositionProcessNoise;

  Eigen::Matrix<float, SENSOR_DIM, SENSOR_DIM> R =
      Eigen::Matrix<float, SENSOR_DIM, SENSOR_DIM>::Identity();
  R.block(0, 0, 12, 12) =
      _R0.block(0, 0, 12, 12) * parameters.footPositionSensorNoise;
  R.block(12, 12, 12, 12) =
      _R0.block(12, 12, 12, 12) * parameters.footVelocitySensorNoise;
  R.block(24, 24, 4, 4) =
      _R0.block(24, 24, 4, 4) * parameters.contactHeightSensorNoise;
  R.block(28, 28, 1, 1) =
      _R0.block(28, 28, 1, 1) * parameters.externalOdometryNoisePosition[0];
  R.block(29, 29, 1, 1) =
      _R0.block(29, 29, 1, 1) * parameters.externalOdometryNoisePosition[1];
  R.block(30, 30, 1, 1) =
      _R0.block(30, 30, 1, 1) * parameters.externalOdometryNoisePosition[2];

  if (inputs.useExternalOdometry) {
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
  Eigen::Vector3f p0, v0;
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];

  Eigen::Matrix<float, 4, 1> trusts = Eigen::Matrix<float, 4, 1>::Zero();
  float h_i = 0.0f;
  for (int i = 0; i < 4; i++) {
    int i1 = 3 * i;
    Eigen::Vector3f p_rel = inputs.footPositionsTrunkFrame.row(i);
    Eigen::Vector3f dp_rel = inputs.footVelocitiesTrunkFrame.row(i);
    Eigen::Vector3f p_f = inputs.bodyToWorldMat * p_rel;
    Eigen::Vector3f dp_f =
        inputs.bodyToWorldMat * (inputs.gyroscope.cross(p_rel) + dp_rel);

    qindex = 6 + i1;
    rindex1 = i1;
    rindex2 = 12 + i1;
    rindex3 = 24 + i;

    float trust = inputs.footContacts(i);

    float high_suspect_number(100);

    Q.block(qindex, qindex, 3, 3) =
        (float(1) + (float(1) - trust) * high_suspect_number) *
        Q.block(qindex, qindex, 3, 3);
    R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
    R.block(rindex2, rindex2, 3, 3) =
        (float(1) + (float(1) - trust) * high_suspect_number) *
        R.block(rindex2, rindex2, 3, 3);
    R(rindex3, rindex3) =
        (float(1) + (float(1) - trust) * high_suspect_number) *
        R(rindex3, rindex3);

    trusts(i) = trust;

    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);

    pzs(i) = (1.0f - trust) * (p0(2) + p_f(2)) +
             trust * inputs.footContactHeights[i];
  }

  Eigen::Matrix<float, SENSOR_DIM, 1> y;
  y << _ps, _vs, pzs, inputs.externalOdometryPosition;
  _xhat = _A * _xhat + _B * inputs.accelerometer;

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

  output.position = _xhat.block(0, 0, 3, 1);
  output.velocityWorld = _xhat.block(3, 0, 3, 1);
  output.velocityBody =
      inputs.bodyToWorldMat.transpose() * _xhat.block(3, 0, 3, 1);
  output.footPositionsWorld = _xhat.block(6, 0, 12, 1);
  output.positionCovariance = _P.block(0, 0, 3, 3);
}

void KalmanFilterObserver::reset() { _xhat.setZero(); }

} // namespace strelka
