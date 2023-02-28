#ifndef A1_STATE_ESITMATOR_H
#define A1_STATE_ESITMATOR_H

#include <common/A1/constants.hpp>
#include <common/A1/kinematics.hpp>
#include <common/constants.hpp>
#include <common/rotation.hpp>
#include <lcm/lcm-cpp.hpp>
#include <messages/a1_lcm_msgs/RobotRawState.hpp>
#include <messages/a1_lcm_msgs/RobotState.hpp>
#include <robots/UnitreeA1.hpp>
#include <state_estimation/KalmanFilterObserver.hpp>
#include <state_estimation/SlowdownEstimator.hpp>

namespace strelka {
static KalmanFilterObserver::KalmanFilterObserverParams
    A1_DEFAULT_KALMAN_FILTER_PARAMS = {
        .dt = 0.001,
        .imuPositionProcessNoise = 0.02,
        .imuVelocityProcessNoise = 0.02,
        .footPositionProcessNoise = 0.002,
        .footPositionSensorNoise = 0.001,
        .footVelocitySensorNoise = 0.1,
        .contactHeightSensorNoise = 0.001,
        .externalOdometryNoisePosition = {0.02, 0.02, 0.09},
};

class A1StateEstimator {
  lcm::LCM lcm;
  KalmanFilterObserver *observer;
  a1_lcm_msgs::RobotState *robotStateMsg;
  lcm::Subscription *sub;

  void update(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
              const a1_lcm_msgs::RobotRawState *messageIn);

  void propagateRobotRawState(const a1_lcm_msgs::RobotRawState *messageIn,
                              a1_lcm_msgs::RobotState *messageOut);

  void fillStateEstimatorData(robots::UnitreeA1 &robot,
                              const KalmanFilterObserver *observer,
                              a1_lcm_msgs::RobotState *messageOut);

public:
  A1StateEstimator();
  void processLoop();

  ~A1StateEstimator();
};

} // namespace strelka

#endif // A1_STATE_ESITMATOR_H