#ifndef A1WBIC_H
#define A1WBIC_H

#include <iostream>
#include <strelka_common/typedefs.hpp>
#include <strelka_common/utilities.hpp>
#include <strelka_control/WholeBodyImpulseController.hpp>
#include <strelka_messages/a1_lcm_msgs/RobotLowCommand.hpp>
#include <strelka_messages/a1_lcm_msgs/RobotState.hpp>
#include <strelka_messages/a1_lcm_msgs/WbicCommand.hpp>
#include <strelka_robots/A1/UnitreeA1.hpp>
#include <strelka_robots/A1/control/A1WbicDynamics.hpp>
#include <strelka_robots/A1/kinematics.hpp>
#include <strelka_state_estimation/SlowdownEstimator.hpp>

namespace strelka {
namespace control {
class A1WBIC {
  lcm::LCM lcm;
  control::WholeBodyImpulseController controller;

  a1_lcm_msgs::RobotLowCommand *lowCommandMessage;
  a1_lcm_msgs::WbicCommand *currentCommandMessage;
  a1_lcm_msgs::RobotState *currentRobotStateMessage;

  bool firstCommandRecieved;
  bool firstStateMessageRecieved;

  ChronoTimePoint lastCommandTimestamp;

  lcm::Subscription *stateSub;
  lcm::Subscription *commandSub;

  void stateHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                   const a1_lcm_msgs::RobotState *messageIn);

  void commandHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                     const a1_lcm_msgs::WbicCommand *commandMsg);

public:
  static constexpr float COMMAND_TIMEOUT_SECONDS = 0.5;
  bool rolloverProtection(robots::Robot &robot);
  bool outputSafetyCheck(WholeBodyImpulseController::WBICOutput &wbicOutput);
  A1WBIC(control::WholeBodyImpulseController::WBICParams &parameters);
  ~A1WBIC();
  void processLoop();
};
} // namespace control
} // namespace strelka
#endif