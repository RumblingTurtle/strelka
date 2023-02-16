#include <control/A1/A1WBIC.hpp>
namespace strelka {

void A1WBIC::stateHandle(const lcm::ReceiveBuffer *rbuf,
                         const std::string &chan,
                         const a1_lcm_msgs::RobotState *messageIn) {
  memcpy(currentState, messageIn, sizeof(a1_lcm_msgs::RobotState));
}

void A1WBIC::commandHandle(const lcm::ReceiveBuffer *rbuf,
                           const std::string &chan,
                           const a1_lcm_msgs::WbicCommand *commandMsg) {

  messages::WBICCommand command(commandMsg);
  strelka::robots::UnitreeA1 robot(currentState);

  strelka::control::WholeBodyImpedanceController::WBICOutput outs;

  controller.update(robot, command, outs);

  for (int motorId = 0; motorId < 12; motorId++) {
    commandMessage->q[motorId] = outs.q[motorId];
    commandMessage->kp[motorId] = constants::A1::POSITION_GAINS[motorId % 3];
    commandMessage->dq[motorId] = outs.dq[motorId];
    commandMessage->kd[motorId] = constants::A1::DAMPING_GAINS[motorId % 3];
    commandMessage->tau[motorId] = outs.tau[motorId];
  }

  lcm.publish("robot_low_command", commandMessage);
}

void A1WBIC::initialize() {
  commandMessage = new a1_lcm_msgs::RobotLowCommand();
  currentState = new a1_lcm_msgs::RobotState();
}

A1WBIC::A1WBIC(control::WholeBodyImpedanceController::WBICParams &parameters)
    : controller(parameters) {
  initialize();
}

void A1WBIC::processLoop() {
  // TODO: split into threads
  stateSub = lcm.subscribe("robot_state", &A1WBIC::stateHandle, this);
  commandSub = lcm.subscribe("wbic_command", &A1WBIC::commandHandle, this);

  while (lcm.handle() == 0) {
  }
}

A1WBIC::~A1WBIC() {
  if (commandMessage)
    delete commandMessage;

  if (currentState)
    delete currentState;

  if (stateSub)
    lcm.unsubscribe(stateSub);

  if (commandSub)
    lcm.unsubscribe(commandSub);
}
} // namespace strelka