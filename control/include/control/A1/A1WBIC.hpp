#ifndef A1WBIC_H
#define A1WBIC_H

#include <common/A1/constants.hpp>
#include <common/A1/kinematics.hpp>
#include <common/typedefs.hpp>
#include <control/WholeBodyImpedanceController.hpp>
#include <messages/a1_lcm_msgs/RobotLowCommand.hpp>
#include <messages/a1_lcm_msgs/RobotState.hpp>
#include <messages/a1_lcm_msgs/WbicCommand.hpp>
#include <robots/UnitreeA1.hpp>
#include <state_estimation/SlowdownEstimator.hpp>

namespace strelka {
static WholeBodyImpedanceController::WBICParams A1_STAND_WBIC_PARAMS = {
    .Kp = {10.0, 10.0, 100.0, 100.0, 100.0, 100.0, 0.0, 0.0, 0.0},
    .Kd = {1.0, 1.0, 10.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0},
    .Kp_kin = {1, 1, 1, 1, 1, 1, 1, 1, 1},
    .floating_W = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
    .rf_W = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    .mu = 0.4,
    .max_fz = 1500,
};

static WholeBodyImpedanceController::WBICParams A1_DEFAULT_WBIC_PARAMS = {
    .Kp = {10.0, 10.0, 100.0, 100.0, 100.0, 100.0, 0.0, 0.0, 0.0},
    .Kd = {1.0, 1.0, 10.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0},
    .Kp_kin = {0, 0, 0, 0.8, 0.8, 0.8, 1, 1, 1},
    .floating_W = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
    .rf_W = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
    .mu = 0.4,
    .max_fz = 1500,
};

class A1WBIC {
  lcm::LCM lcm;
  WholeBodyImpedanceController controller;

  a1_lcm_msgs::RobotLowCommand *commandMessage;
  a1_lcm_msgs::RobotState *currentState;

  lcm::Subscription *stateSub;
  lcm::Subscription *commandSub;

  void stateHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                   const a1_lcm_msgs::RobotState *messageIn) {
    memcpy(currentState, messageIn, sizeof(a1_lcm_msgs::RobotState));
  }

  void commandHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                     const a1_lcm_msgs::WbicCommand *commandMsg) {

    strelka::control::WBIC::WBICCommand command(commandMsg);
    strelka::robots::UnitreeA1 robot(currentState);

    strelka::WholeBodyImpedanceController::WBICOutput outs;

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

  void initialize() {
    commandMessage = new a1_lcm_msgs::RobotLowCommand();
    currentState = new a1_lcm_msgs::RobotState();
  }

public:
  A1WBIC(WholeBodyImpedanceController::WBICParams &parameters)
      : controller(parameters) {
    initialize();
  }

  void processLoop() {
    // TODO: split into threads
    stateSub = lcm.subscribe("robot_state", &A1WBIC::stateHandle, this);
    commandSub = lcm.subscribe("wbic_command", &A1WBIC::commandHandle, this);

    while (lcm.handle() == 0) {
    }
  }

  ~A1WBIC() {
    if (commandMessage)
      delete commandMessage;

    if (currentState)
      delete currentState;

    if (stateSub)
      lcm.unsubscribe(stateSub);

    if (commandSub)
      lcm.unsubscribe(commandSub);
  }
};

} // namespace strelka
#endif