#ifndef KALMAN_FILTER_OBSERVER_H
#define KALMAN_FILTER_OBSERVER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>

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

  struct KalmanFilterObserverInput {
    Eigen::Matrix3f bodyToWorldMat; // BodyToWorld
    Eigen::Vector4f footContacts;
    Eigen::Matrix<float, 4, 3> footPositionsTrunkFrame;  // Trunk frame
    Eigen::Matrix<float, 4, 3> footVelocitiesTrunkFrame; // Trunk frame
    Eigen::Vector3f gyroscope;                // angular rates in body frame
    Eigen::Vector3f accelerometer;            // world frame base acceleration
    Eigen::Vector4f footContactHeights;       // world frame
    Eigen::Vector3f externalOdometryPosition; // world frame
    bool useExternalOdometry;
  };

  struct KalmanFilterObserverOutput {
    Eigen::Vector3f position;
    Eigen::Vector3f velocityBody;
    Eigen::Vector3f velocityWorld;
    Eigen::Matrix<float, 12, 1> footPositionsWorld;
    Eigen::Matrix3f positionCovariance;
  };

  KalmanFilterObserver();
  KalmanFilterObserver(KalmanFilterObserverParams &);

  void setParameters(KalmanFilterObserverParams &);

  void update(KalmanFilterObserverInput &inputs,
              KalmanFilterObserverOutput &output);
  void reset();
};

}; // namespace strelka

#endif // KALMAN_FILTER_OBSERVER_H