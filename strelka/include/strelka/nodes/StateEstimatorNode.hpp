#ifndef STATE_ESITMATOR_NODE_H
#define STATE_ESITMATOR_NODE_H

#include <lcm/lcm-cpp.hpp>
#include <strelka/common/constants.hpp>
#include <strelka/common/rotation.hpp>
#include <strelka/filters/MovingWindowFilter3D.hpp>
#include <strelka/state_estimation/KalmanFilterObserver.hpp>
#include <strelka/state_estimation/SlowdownEstimator.hpp>
#include <strelka_lcm_headers/RobotRawState.hpp>
#include <strelka_lcm_headers/RobotState.hpp>

#include <strelka/robots/Robots.hpp>
namespace strelka {
namespace state_estimation {
constexpr float STATE_ESTIMATOR_WARMUP_TIME = 3.0f; // Seconds
template <class RobotClass> class StateEstimatorNode {
  lcm::LCM lcm;
  KalmanFilterObserver *observer;
  strelka_lcm_headers::RobotState *robotStateMsg;
  lcm::Subscription *sub;

  float filterWarmupTime;
  float prevTick;
  bool firstRun;
  Vec4<float> contactHeightEstimates;
  Vec4<bool> previousContacts;

  static constexpr float HEIGHT_DIFF_THRESHOLD = 0.03; // cm
  filters::MovingWindowFilter3D<float> velocityFilter{};

  void update(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
              const strelka_lcm_headers::RobotRawState *messageIn);

  void
  propagateRobotRawState(const strelka_lcm_headers::RobotRawState *messageIn,
                         strelka_lcm_headers::RobotState *messageOut);

  void updateFootContactHeights(robots::Robot &robot);
  void fillStateEstimatorData(robots::Robot &robot,
                              const KalmanFilterObserver *observer,
                              strelka_lcm_headers::RobotState *messageOut);

public:
  StateEstimatorNode();
  void processLoop();
  bool handle();
  ~StateEstimatorNode();
};
} // namespace state_estimation
} // namespace strelka

#endif // STATE_ESITMATOR_NODE_H