#include <strelka_robots/A1/control/A1WBIC.hpp>
namespace strelka {
namespace control {

A1WBIC::A1WBIC(control::WholeBodyImpulseController::WBICParams &parameters)
    // Dummy static robot instance is used here only to fetch some constants
    : controller(robots::UnitreeA1::createDummyA1RobotWithRawState(),
                 parameters),
      firstCommandRecieved(false), lastCommandTimestamp(getWallTime()),
      firstStateMessageRecieved(false) {

  lowCommandMessage = new strelka_lcm_headers::RobotLowCommand();
  currentCommandMessage = new strelka_lcm_headers::WbicCommand();
  currentRobotStateMessage = new strelka_lcm_headers::RobotState();

  stateSub = lcm.subscribe(constants::ROBOT_STATE_TOPIC_NAME,
                           &A1WBIC::stateHandle, this);
  commandSub = lcm.subscribe(constants::WBIC_COMMAND_TOPIC_NAME,
                             &A1WBIC::commandHandle, this);
  stateSub->setQueueCapacity(1);
  commandSub->setQueueCapacity(1);
}

void A1WBIC::stateHandle(const lcm::ReceiveBuffer *rbuf,
                         const std::string &chan,
                         const strelka_lcm_headers::RobotState *messageIn) {
  memcpy(currentRobotStateMessage, messageIn,
         sizeof(strelka_lcm_headers::RobotState));
  firstStateMessageRecieved = true;
}

void A1WBIC::commandHandle(const lcm::ReceiveBuffer *rbuf,
                           const std::string &chan,
                           const strelka_lcm_headers::WbicCommand *commandMsg) {
  memcpy(currentCommandMessage, commandMsg,
         sizeof(strelka_lcm_headers::WbicCommand));
  if (!firstCommandRecieved) {
    firstCommandRecieved = true;
  }
  lastCommandTimestamp = getWallTime();
}

bool A1WBIC::rolloverProtection(robots::Robot &robot) {
  Vec3<float> currentRPY = robot.currentRPY();

  bool robotRolledOver = currentRPY(0) > M_PI_2 || currentRPY(1) > M_PI_2;
  if (robotRolledOver) {
    std::cout << "WBIC: Rollover protection \n RPY " << currentRPY.transpose()
              << std::endl;
    return true;
  }
  return false;
}

bool A1WBIC::outputSafetyCheck(
    WholeBodyImpulseController::WBICOutput &wbicOutput) {
  if (wbicOutput.q.hasNaN()) {
    std::cout << "WBIC: Encountered nan value in desired angles:\n"
              << wbicOutput.q.transpose() << std::endl;
    return true;
  }

  if (wbicOutput.dq.hasNaN()) {
    std::cout << "WBIC: Encountered nan value in desired joint velocities:\n"
              << wbicOutput.q.transpose() << std::endl;
    return true;
  }

  if (wbicOutput.tau.hasNaN()) {
    std::cout << "WBIC: Encountered nan value in desired torques:\n"
              << wbicOutput.q.transpose() << std::endl;
    return true;
  }
  return false;
}

bool A1WBIC::handle() {
  static WholeBodyImpulseController::WBICOutput wbicOutput;
  if (0 != lcm.handle()) {
    return false;
  }

  float commandDeltaTime =
      timePointDiffInSeconds(getWallTime(), lastCommandTimestamp);

  if (firstCommandRecieved && commandDeltaTime > COMMAND_TIMEOUT_SECONDS) {
    std::cout << "WBIC: " << constants::WBIC_COMMAND_TOPIC_NAME
              << " topic timeout. Last message recieved " << commandDeltaTime
              << " seconds ago." << std::endl;
    return false;
  }

  if (firstCommandRecieved && firstStateMessageRecieved) {
    messages::WBICCommand currentCommand{currentCommandMessage};
    strelka::robots::UnitreeA1 robot(currentRobotStateMessage);

    if (rolloverProtection(robot)) {
      return false;
    }

    if (currentCommandMessage->stop) {
      std::cout << "WBIC: Recieved stop flag in command message." << std::endl;
      return false;
    }

    controller.update(robot, currentCommand, wbicOutput);

    if (outputSafetyCheck(wbicOutput)) {
      return false;
    }

    for (int motorId = 0; motorId < 12; motorId++) {
      lowCommandMessage->q[motorId] = wbicOutput.q[motorId];
      lowCommandMessage->dq[motorId] = wbicOutput.dq[motorId];
      lowCommandMessage->kp[motorId] =
          A1::constants::POSITION_GAINS[motorId % 3];
      lowCommandMessage->kd[motorId] =
          A1::constants::DAMPING_GAINS[motorId % 3];
      lowCommandMessage->tau[motorId] = wbicOutput.tau[motorId];
    }

    lcm.publish("robot_low_command", lowCommandMessage);
  }
  return true;
}

void A1WBIC::processLoop() {
  while (handle()) {
  }
}

A1WBIC::~A1WBIC() {
  delete lowCommandMessage;
  delete currentRobotStateMessage;
  delete currentCommandMessage;

  if (stateSub) {
    lcm.unsubscribe(stateSub);
  }

  if (commandSub) {
    lcm.unsubscribe(commandSub);
  }
}
} // namespace control
} // namespace strelka