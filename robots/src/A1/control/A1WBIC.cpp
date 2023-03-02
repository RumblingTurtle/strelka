#include <robots/A1/control/A1WBIC.hpp>

namespace strelka {
namespace control {
void A1WBIC::stateHandle(const lcm::ReceiveBuffer *rbuf,
                         const std::string &chan,
                         const a1_lcm_msgs::RobotState *messageIn) {
  static strelka::control::WholeBodyImpulseController::WBICOutput outs;
  strelka::robots::UnitreeA1 robot(messageIn);

  if (firstCommandRecieved) {
    messages::WBICCommand currentCommand{currentCommandMessage};
    controller.update(robot, currentCommand, outs);

    for (int motorId = 0; motorId < 12; motorId++) {
      lowCommandMessage->q[motorId] = outs.q[motorId];
      lowCommandMessage->dq[motorId] = outs.dq[motorId];
      lowCommandMessage->kp[motorId] =
          A1::constants::POSITION_GAINS[motorId % 3];
      lowCommandMessage->kd[motorId] =
          A1::constants::DAMPING_GAINS[motorId % 3];
      lowCommandMessage->tau[motorId] = outs.tau[motorId];
    }

    lcm.publish("robot_low_command", lowCommandMessage);
  }
}

void A1WBIC::commandHandle(const lcm::ReceiveBuffer *rbuf,
                           const std::string &chan,
                           const a1_lcm_msgs::WbicCommand *commandMsg) {
  memcpy(currentCommandMessage, commandMsg, sizeof(a1_lcm_msgs::WbicCommand));
  firstCommandRecieved = true;
  lastCommandTimestamp = getWallTime();
}

void A1WBIC::initialize() {
  lowCommandMessage = new a1_lcm_msgs::RobotLowCommand();
  currentCommandMessage = new a1_lcm_msgs::WbicCommand();
}

A1WBIC::A1WBIC(control::WholeBodyImpulseController::WBICParams &parameters)
    : controller(buildA1<float>().buildModel(), parameters),
      firstCommandRecieved(false), lastCommandTimestamp(getWallTime()) {
  initialize();
}

void A1WBIC::processLoop() {
  // TODO: split into threads
  stateSub = lcm.subscribe("robot_state", &A1WBIC::stateHandle, this);
  commandSub = lcm.subscribe("wbic_command", &A1WBIC::commandHandle, this);
  stateSub->setQueueCapacity(1);
  commandSub->setQueueCapacity(1);
  while (true) {
    if (0 != lcm.handle()) {
      break;
    }

    bool commandTimeout =
        timePointDiffInSeconds(getWallTime(), lastCommandTimestamp) > 0.5;

    if (firstCommandRecieved && commandTimeout) {
      break;
    }
  }
}

A1WBIC::~A1WBIC() {
  if (lowCommandMessage) {
    delete lowCommandMessage;
  }
  if (currentCommandMessage) {
    delete currentCommandMessage;
  }
  if (stateSub) {
    lcm.unsubscribe(stateSub);
  }

  if (commandSub) {
    lcm.unsubscribe(commandSub);
  }
}
} // namespace control
} // namespace strelka