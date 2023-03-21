#ifndef A1WBIC_H
#define A1WBIC_H

#include <iostream>
#include <strelka/common/typedefs.hpp>
#include <strelka/common/utilities.hpp>
#include <strelka/control/WholeBodyImpulseController.hpp>
#include <strelka/state_estimation/SlowdownEstimator.hpp>
#include <strelka_lcm_headers/RobotLowCommand.hpp>
#include <strelka_lcm_headers/RobotState.hpp>
#include <strelka_lcm_headers/WbicCommand.hpp>
#include <strelka_robots/A1/UnitreeA1.hpp>
#include <strelka_robots/A1/kinematics.hpp>

namespace strelka {
namespace control {
class A1WBIC {
  lcm::LCM lcm;
  control::WholeBodyImpulseController controller;

  strelka_lcm_headers::RobotLowCommand *lowCommandMessage;
  strelka_lcm_headers::WbicCommand *currentCommandMessage;
  strelka_lcm_headers::RobotState *currentRobotStateMessage;

  bool firstCommandRecieved;
  bool firstStateMessageRecieved;

  ChronoTimePoint lastCommandTimestamp;

  lcm::Subscription *stateSub;
  lcm::Subscription *commandSub;

  void stateHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                   const strelka_lcm_headers::RobotState *messageIn);

  void commandHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                     const strelka_lcm_headers::WbicCommand *commandMsg);

public:
  static constexpr float COMMAND_TIMEOUT_SECONDS = 0.5;
  bool rolloverProtection(robots::Robot &robot);
  bool outputSafetyCheck(WholeBodyImpulseController::WBICOutput &wbicOutput);
  A1WBIC(control::WholeBodyImpulseController::WBICParams &parameters);
  ~A1WBIC();
  bool handle();
  void processLoop();
};
} // namespace control
} // namespace strelka
#endif