#include <strelka_robots/A1/control/A1WBIC.hpp>
#include <strelka_robots/A1/control/A1WbicDynamics.hpp>

namespace strelka {
namespace control {

A1WBIC::A1WBIC(control::WholeBodyImpulseController::WBICParams &parameters)
    : controller(buildA1<float>().buildModel(), parameters),
      firstCommandRecieved(false), lastCommandTimestamp(getWallTime()),
      firstStateMessageRecieved(false) {

  lowCommandMessage = new a1_lcm_msgs::RobotLowCommand();
  currentCommandMessage = new a1_lcm_msgs::WbicCommand();
  currentRobotStateMessage = new a1_lcm_msgs::RobotState();
}

void A1WBIC::stateHandle(const lcm::ReceiveBuffer *rbuf,
                         const std::string &chan,
                         const a1_lcm_msgs::RobotState *messageIn) {
  memcpy(currentRobotStateMessage, messageIn, sizeof(a1_lcm_msgs::RobotState));
  firstStateMessageRecieved = true;
}

void A1WBIC::commandHandle(const lcm::ReceiveBuffer *rbuf,
                           const std::string &chan,
                           const a1_lcm_msgs::WbicCommand *commandMsg) {
  memcpy(currentCommandMessage, commandMsg, sizeof(a1_lcm_msgs::WbicCommand));
  firstCommandRecieved = true;
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

void A1WBIC::processLoop() {
  static WholeBodyImpulseController::WBICOutput wbicOutput;

  stateSub = lcm.subscribe(A1::constants::ROBOT_STATE_TOPIC_NAME,
                           &A1WBIC::stateHandle, this);
  commandSub = lcm.subscribe("wbic_command", &A1WBIC::commandHandle, this);
  stateSub->setQueueCapacity(1);
  commandSub->setQueueCapacity(1);
  while (true) {
    if (0 != lcm.handle()) {
      break;
    }

    float commandDeltaTime =
        timePointDiffInSeconds(getWallTime(), lastCommandTimestamp);

    if (firstCommandRecieved && commandDeltaTime > COMMAND_TIMEOUT_SECONDS) {
      std::cout << "WBIC: Command topic timeout. Last message recieved "
                << commandDeltaTime << " seconds ago." << std::endl;
      break;
    }

    if (firstCommandRecieved && firstStateMessageRecieved) {
      messages::WBICCommand currentCommand{currentCommandMessage};
      strelka::robots::UnitreeA1 robot(currentRobotStateMessage);

      if (rolloverProtection(robot)) {
        break;
      }

      if (currentCommandMessage->stop) {
        std::cout << "WBIC: Recieved stop flag in command message."
                  << std::endl;
        break;
      }

      controller.update(robot, currentCommand, wbicOutput);

      if (outputSafetyCheck(wbicOutput)) {
        break;
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