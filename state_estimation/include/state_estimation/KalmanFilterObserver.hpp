#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>

#ifndef KALMAN_FILTER_OBSERVER_H
#define KALMAN_FILTER_OBSERVER_H

#define STATE_DIM 18
#define SENSOR_DIM 31

namespace strelka {

class KalmanFilterObserver {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

public:
  struct KalmanFilterObserverParams {
    float dt;

    float imuPositionProcessNoise;
    float imuVelocityProcessNoise;

    float footPositionProcessNoise;

    float footPositionSensorNoise;
    float footVelocitySensorNoise;

    float contactHeightSensorNoise;

    Eigen::VectorXf initialState;
    Eigen::Vector3f externalOdometryNoisePosition;
  } parameters;

  struct KalmanFilterObserverInput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix3f bodyToWorldMat; // BodyToWorld
    Eigen::Matrix<float, 4, 1> footContacts;
    Eigen::Matrix<float, 4, 3> footPositions;  // COM frame
    Eigen::Matrix<float, 4, 3> footVelocities; // COM frame
    Eigen::Vector3f gyroscope;                 // angular rates in body frame
    Eigen::Vector3f accelerometer;             // world frame base acceleration
    Eigen::Matrix<float, 4, 1> footContactHeights; // world frame
    Eigen::Vector3f externalOdometryPosition;      // world frame
    bool useExternalOdometry;
  };

  struct KalmanFilterObserverOutput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3f position;
    Eigen::Vector3f velocityBody;
    Eigen::Vector3f velocityWorld;
    Eigen::Matrix<float, 12, 1> footPositionsWorld;
    Eigen::Matrix3f positionCovariance;
  };

  KalmanFilterObserver(KalmanFilterObserverParams &);
  KalmanFilterObserverOutput update(KalmanFilterObserverInput &);
  void reset();
};

}; // namespace strelka

#endif // KALMAN_FILTER_OBSERVER_H