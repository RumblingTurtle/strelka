#ifndef WBIC_NODE_H
#define WBIC_NODE_H

#include <iostream>
#include <strelka/common/constants.hpp>
#include <strelka/common/typedefs.hpp>
#include <strelka/common/utilities.hpp>
#include <strelka/control/WholeBodyImpulseController.hpp>
#include <strelka/state_estimation/SlowdownEstimator.hpp>

#include <strelka_lcm_headers/RobotLowCommand.hpp>
#include <strelka_lcm_headers/RobotState.hpp>
#include <strelka_lcm_headers/WbicCommand.hpp>

#include <strelka/robots/Robots.hpp>

namespace strelka {
namespace control {
template <class RobotClass> class WBICNode {
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
  WBICNode(robots::Robot &robot,
           control::WholeBodyImpulseController::WBICParams &parameters);
  ~WBICNode();
  bool handle();
  void processLoop();
};
} // namespace control
} // namespace strelka
#endif // WBIC_NODE_H