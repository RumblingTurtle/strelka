#ifndef KALMAN_FILTER_OBSERVER_H
#define KALMAN_FILTER_OBSERVER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
#include <strelka/common/macros.hpp>
#include <strelka/robots/Robot.hpp>

namespace strelka {
namespace state_estimation {
class UninitializedKalmanFilter : public std::exception {
public:
  const char *what() {
    return "KalmanFilterObserver haven't been provided with initial "
           "parameters. Call setParameters(KalmanFilterObserverParams &) or "
           "use constructor with KalmanFilterObserverParams";
  }
};

class KalmanFilterObserver {
  constexpr static int STATE_DIM = 18;
  constexpr static int SENSOR_DIM = 31;

  Eigen::Matrix<float, STATE_DIM, 1> _xhat;
  Vec12<float> _ps;
  Vec12<float> _vs;
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
  Vec3<float> _position;
  Vec3<float> _velocityBody;
  Vec3<float> _velocityWorld;
  Vec12<float> _footPositionsWorld;
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

    Vec3<float> externalOdometryNoisePosition;
  } parameters;

  Vec3<float> position() const;
  Vec3<float> velocityBody() const;
  Vec3<float> velocityWorld() const;
  Vec12<float> footPositionsWorld() const;
  Eigen::Matrix3f positionCovariance() const;

  KalmanFilterObserver();
  KalmanFilterObserver(KalmanFilterObserverParams &);

  void setParameters(KalmanFilterObserverParams &);

  void update(robots::Robot &robot, bool useExternalOdometry,
              Vec3<float> externalOdometryPosition, Vec4<float> contactHeights);

  void reset();
};

static KalmanFilterObserver::KalmanFilterObserverParams
    DEFAULT_KALMAN_FILTER_PARAMS = {
        .dt = 0.001,
        .imuPositionProcessNoise = 0.02,
        .imuVelocityProcessNoise = 0.02,
        .footPositionProcessNoise = 0.002,
        .footPositionSensorNoise = 0.001,
        .footVelocitySensorNoise = 0.1,
        .contactHeightSensorNoise = 0.001,
        .externalOdometryNoisePosition = {0.02, 0.02, 0.09},
};
} // namespace state_estimation
}; // namespace strelka

#endif // KALMAN_FILTER_OBSERVER_H