#include <strelka/nodes/WBICNode.hpp>

#include <strelka/robots/A1/UnitreeA1.hpp>
namespace strelka {
namespace control {

template <class RobotClass>
WBICNode<RobotClass>::WBICNode(
    control::WholeBodyImpulseController::WBICParams &parameters)
    // Dummy static robot instance is used here only to fetch some constants
    : robotInstance(), controller(robotInstance, parameters),
      firstCommandRecieved(false), lastCommandTimestamp(getWallTime()),
      firstStateMessageRecieved(false) {

  lowCommandMessage = new strelka_lcm_headers::RobotLowCommand();
  currentCommandMessage = new strelka_lcm_headers::WbicCommand();
  currentRobotStateMessage = new strelka_lcm_headers::RobotState();

  stateSub = lcm.subscribe(constants::ROBOT_STATE_TOPIC_NAME,
                           &WBICNode::stateHandle, this);
  commandSub = lcm.subscribe(constants::WBIC_COMMAND_TOPIC_NAME,
                             &WBICNode::commandHandle, this);
  stateSub->setQueueCapacity(1);
  commandSub->setQueueCapacity(1);
}

template <class RobotClass>
void WBICNode<RobotClass>::stateHandle(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const strelka_lcm_headers::RobotState *messageIn) {
  memcpy(currentRobotStateMessage, messageIn,
         sizeof(strelka_lcm_headers::RobotState));
  firstStateMessageRecieved = true;
}

template <class RobotClass>
void WBICNode<RobotClass>::commandHandle(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const strelka_lcm_headers::WbicCommand *commandMsg) {
  memcpy(currentCommandMessage, commandMsg,
         sizeof(strelka_lcm_headers::WbicCommand));
  if (!firstCommandRecieved) {
    firstCommandRecieved = true;
  }
  lastCommandTimestamp = getWallTime();
}

template <class RobotClass>
bool WBICNode<RobotClass>::rolloverProtection(robots::Robot &robot) {
  Vec3<float> bodyToWorldRPY = robot.bodyToWorldRPY();

  bool robotRolledOver =
      bodyToWorldRPY(0) > M_PI_2 || bodyToWorldRPY(1) > M_PI_2;
  if (robotRolledOver) {
    std::cout << "WBIC: Rollover protection \n RPY "
              << bodyToWorldRPY.transpose() << std::endl;
    return true;
  }
  return false;
}

template <class RobotClass>
bool WBICNode<RobotClass>::outputSafetyCheck(
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

template <class RobotClass> bool WBICNode<RobotClass>::handle() {
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
    RobotClass robot(currentRobotStateMessage);

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

    Vec3<float> KP = robot.positionGains();
    Vec3<float> KD = robot.dampingGains();

    for (int motorId = 0; motorId < 12; motorId++) {
      lowCommandMessage->q[motorId] = wbicOutput.q[motorId];
      lowCommandMessage->dq[motorId] = wbicOutput.dq[motorId];
      lowCommandMessage->tau[motorId] = wbicOutput.tau[motorId];
      lowCommandMessage->kp[motorId] = KP[motorId % 3];
      lowCommandMessage->kd[motorId] = KD[motorId % 3];
    }

    lcm.publish("robot_low_command", lowCommandMessage);
  }
  return true;
}

template <class RobotClass> void WBICNode<RobotClass>::processLoop() {
  while (handle()) {
  }
}

template <class RobotClass> WBICNode<RobotClass>::~WBICNode() {
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
#define WBIC_NODE_HEADER
#include <strelka/robots/RobotRegistry.hpp>
} // namespace strelka