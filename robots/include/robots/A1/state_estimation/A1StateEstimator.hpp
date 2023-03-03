#ifndef A1_STATE_ESITMATOR_H
#define A1_STATE_ESITMATOR_H

#include <common/constants.hpp>
#include <common/rotation.hpp>
#include <filters/MovingWindowFilter3D.hpp>
#include <lcm/lcm-cpp.hpp>
#include <messages/a1_lcm_msgs/RobotRawState.hpp>
#include <messages/a1_lcm_msgs/RobotState.hpp>
#include <robots/A1/UnitreeA1.hpp>
#include <robots/A1/constants.hpp>
#include <robots/A1/kinematics.hpp>
#include <state_estimation/KalmanFilterObserver.hpp>
#include <state_estimation/SlowdownEstimator.hpp>
namespace strelka {
namespace state_estimation {
class A1StateEstimator {
  lcm::LCM lcm;
  KalmanFilterObserver *observer;
  a1_lcm_msgs::RobotState *robotStateMsg;
  lcm::Subscription *sub;

  bool firstRun;
  Vec4<float> contactHeightEstimates;
  Vec4<bool> previousContacts;

  static constexpr float HEIGHT_DIFF_THRESHOLD = 0.03; // cm
  filters::MovingWindowFilter3D<float> velocityFilter{};

  void update(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
              const a1_lcm_msgs::RobotRawState *messageIn);

  void propagateRobotRawState(const a1_lcm_msgs::RobotRawState *messageIn,
                              a1_lcm_msgs::RobotState *messageOut);
  void updateFootContactHeights(robots::UnitreeA1 &robot);
  void fillStateEstimatorData(robots::UnitreeA1 &robot,
                              const KalmanFilterObserver *observer,
                              a1_lcm_msgs::RobotState *messageOut);

public:
  A1StateEstimator();
  void processLoop();

  ~A1StateEstimator();
};
} // namespace state_estimation
} // namespace strelka

#endif // A1_STATE_ESITMATOR_H