#include <strelka_robots/A1/control/A1LocalPlanner.hpp>

namespace strelka {
namespace control {

A1LocalPlanner::A1LocalPlanner(Gait initialGait)
    : prevTick(-1), localPlanner(initialGait, A1::constants::MPC_BODY_MASS,
                                 A1::constants::MPC_BODY_INERTIA),
      lastCommandTimestamp(getWallTime()), firstCommandRecieved(false) {
  setupProcessLoop();
}

A1LocalPlanner::A1LocalPlanner(std::shared_ptr<FootholdPlanner> footPlanner)
    : prevTick(-1), localPlanner(footPlanner, A1::constants::MPC_BODY_MASS,
                                 A1::constants::MPC_BODY_INERTIA),
      lastCommandTimestamp(getWallTime()), firstCommandRecieved(false) {
  setupProcessLoop();
}

A1LocalPlanner::~A1LocalPlanner() {
  delete wbicCommand;
  delete highCommand;
  lcm.unsubscribe(stateSub);
  lcm.unsubscribe(commandSub);
}

void A1LocalPlanner::stateHandle(const lcm::ReceiveBuffer *rbuf,
                                 const std::string &chan,
                                 const a1_lcm_msgs::RobotState *messageIn) {

  if (!firstCommandRecieved) {
    return;
  }
  messages::HighLevelCommand command{highCommand};
  robots::UnitreeA1 robot{messageIn};

  float dt = messageIn->tick;

  if (prevTick == -1) {
    dt = 0.001;
  } else {
    dt -= prevTick;
  }

  prevTick = messageIn->tick;
  if (dt == 0) {
    // NOTE: happens a couple times at the start for no reason. Might be a
    // gazebo broadcaster issue
    return;
  }

  localPlanner.update(robot, command, dt);

  FOR_EACH_LEG {
    wbicCommand->footState[LEG_ID] = localPlanner.footState(LEG_ID);
  }

  memcpy(wbicCommand->rpy, localPlanner.desiredRpy().data(), sizeof(float) * 3);
  memcpy(wbicCommand->pBody, localPlanner.desiredPositionBody().data(),
         sizeof(float) * 3);
  memcpy(wbicCommand->angularVelocity,
         localPlanner.desiredAngularVelocity().data(), sizeof(float) * 3);
  memcpy(wbicCommand->vBody, localPlanner.desiredVelocityBody().data(),
         sizeof(float) * 3);
  memcpy(wbicCommand->aBody, localPlanner.desiredAccelerationBody().data(),
         sizeof(float) * 3);

  memcpy(wbicCommand->mpcForces, localPlanner.mpcForces().data(),
         sizeof(float) * 12);
  memcpy(wbicCommand->pFoot, localPlanner.desiredFootPositions().data(),
         sizeof(float) * 12);
  memcpy(wbicCommand->vFoot, localPlanner.desiredFootVelocities().data(),
         sizeof(float) * 12);
  memcpy(wbicCommand->aFoot, localPlanner.desiredFootAccelerations().data(),
         sizeof(float) * 12);

  wbicCommand->stop = 0;

  lcm.publish(A1::constants::WBIC_COMMAND_TOPIC_NAME, wbicCommand);
}

void A1LocalPlanner::commandHandle(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const a1_lcm_msgs::HighLevelCommand *commandMsg) {
  memcpy(highCommand, commandMsg, sizeof(a1_lcm_msgs::HighLevelCommand));
  if (!firstCommandRecieved) {
    firstCommandRecieved = true;
  }

  lastCommandTimestamp = getWallTime();
}

void A1LocalPlanner::setupProcessLoop() {
  wbicCommand = new a1_lcm_msgs::WbicCommand();
  highCommand = new a1_lcm_msgs::HighLevelCommand();
  stateSub = lcm.subscribe(A1::constants::ROBOT_STATE_TOPIC_NAME,
                           &A1LocalPlanner::stateHandle, this);
  stateSub->setQueueCapacity(1);

  commandSub = lcm.subscribe(A1::constants::HIGH_LEVEL_COMMAND_TOPIC_NAME,
                             &A1LocalPlanner::commandHandle, this);
  commandSub->setQueueCapacity(1);
}

bool A1LocalPlanner::handle() {
  float commandDeltaTime =
      timePointDiffInSeconds(getWallTime(), lastCommandTimestamp);

  if (firstCommandRecieved && commandDeltaTime > COMMAND_TIMEOUT_SECONDS) {
    std::cout << "A1LocalPlanner: "
              << A1::constants::HIGH_LEVEL_COMMAND_TOPIC_NAME
              << " topic timeout. Last message recieved " << commandDeltaTime
              << " seconds ago." << std::endl;
    return false;
  }
  return lcm.handle() == 0;
}

void A1LocalPlanner::processLoop() {
  while (handle())
    ;
}
} // namespace control
} // namespace strelka
