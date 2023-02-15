#ifndef KALMAN_FILTER_OBSERVER_H
#define KALMAN_FILTER_OBSERVER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
#include <robots/Robot.hpp>

#define STATE_DIM 18
#define SENSOR_DIM 31

namespace strelka {
class UninitializedKalmanFilter : std::exception {
  const char *what() {
    return "KalmanFilterObserver haven't been provided with initial "
           "parameters. Call setParameters(KalmanFilterObserverParams &) or "
           "use constructor with KalmanFilterObserverParams";
  }
};

class KalmanFilterObserver {
  Eigen::Matrix<float, STATE_DIM, 1> _xhat;
  Eigen::Matrix<float, 12, 1> _ps;
  Eigen::Matrix<float, 12, 1> _vs;
  Eigen::Matrix<float, STATE_DIM, STATE_DIM> _A;
  Eigen::Matrix<float, STATE_DIM, STATE_DIM> _Q0;
  Eigen::Matrix<float, STATE_DIM, STATE_DIM> _P;
  Eigen::Matrix<float, SENSOR_DIM, SENSOR_DIM> _R0;
  Eigen::Matrix<float, STATE_DIM, 3> _B;
  Eigen::Matrix<float, SENSOR_DIM, STATE_DIM> _C;
  Eigen::Matrix<float, STATE_DIM, 1> state;
  Eigen::Matrix<float, STATE_DIM, 1> covariance;
  bool initialized;

  /*Outputs*/
  Eigen::Vector3f _position;
  Eigen::Vector3f _velocityBody;
  Eigen::Vector3f _velocityWorld;
  Eigen::Matrix<float, 12, 1> _footPositionsWorld;
  Eigen::Matrix3f _positionCovariance;

  void initialize();

public:
  struct KalmanFilterObserverParams {
    float dt;

    float imuPositionProcessNoise;
    float imuVelocityProcessNoise;

    float footPositionProcessNoise;

    float footPositionSensorNoise;
    float footVelocitySensorNoise;

    float contactHeightSensorNoise;

    Eigen::Vector3f externalOdometryNoisePosition;
  } parameters;

  Eigen::Vector3f position() const;
  Eigen::Vector3f velocityBody() const;
  Eigen::Vector3f velocityWorld() const;
  Eigen::Matrix<float, 12, 1> footPositionsWorld() const;
  Eigen::Matrix3f positionCovariance() const;

  KalmanFilterObserver();
  KalmanFilterObserver(KalmanFilterObserverParams &);

  void setParameters(KalmanFilterObserverParams &);

  void update(robots::Robot &robot, bool useExternalOdometry,
              Vec3<float> externalOdometryPosition);

  void reset();
};

}; // namespace strelka

#endif // KALMAN_FILTER_OBSERVER_H