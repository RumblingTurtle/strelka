#include <strelka_robots/LocalPlannerNode.hpp>

#include <strelka_robots/A1/UnitreeA1.hpp>

namespace strelka {
namespace control {

template <class RobotClass>
LocalPlannerNode<RobotClass>::LocalPlannerNode(Gait initialGait,
                                               float robotMass,
                                               Mat3<float> rotationalInertia,
                                               float stepDt, int horizonSteps)
    : prevTick(-1), localPlanner(initialGait, robotMass, rotationalInertia,
                                 stepDt, horizonSteps),
      lastCommandTimestamp(getWallTime()), firstCommandRecieved(false) {
  setupProcessLoop();
}

template <class RobotClass>
LocalPlannerNode<RobotClass>::LocalPlannerNode(
    std::shared_ptr<FootholdPlanner> footPlanner, float robotMass,
    Mat3<float> rotationalInertia, float stepDt, int horizonSteps)
    : prevTick(-1), localPlanner(footPlanner, robotMass, rotationalInertia,
                                 stepDt, horizonSteps),
      lastCommandTimestamp(getWallTime()), firstCommandRecieved(false) {
  setupProcessLoop();
}

template <class RobotClass> LocalPlannerNode<RobotClass>::~LocalPlannerNode() {
  delete wbicCommand;
  delete highCommand;
  lcm.unsubscribe(stateSub);
  lcm.unsubscribe(commandSub);
}

template <class RobotClass>
void LocalPlannerNode<RobotClass>::stateHandle(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const strelka_lcm_headers::RobotState *messageIn) {

  if (!firstCommandRecieved) {
    return;
  }

  messages::HighLevelCommand command{highCommand};
  RobotClass robot{messageIn};

  float dt = messageIn->tick;

  if (prevTick == -1) {
    dt = 0.001;
  } else {
    dt -= prevTick;
  }

  prevTick = messageIn->tick;
  if (dt == 0) {
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

  lcm.publish(constants::WBIC_COMMAND_TOPIC_NAME, wbicCommand);
}

template <class RobotClass>
void LocalPlannerNode<RobotClass>::commandHandle(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const strelka_lcm_headers::HighLevelCommand *commandMsg) {
  memcpy(highCommand, commandMsg,
         sizeof(strelka_lcm_headers::HighLevelCommand));
  if (!firstCommandRecieved) {
    firstCommandRecieved = true;
  }

  lastCommandTimestamp = getWallTime();
}

template <class RobotClass>
void LocalPlannerNode<RobotClass>::setupProcessLoop() {
  wbicCommand = new strelka_lcm_headers::WbicCommand();
  highCommand = new strelka_lcm_headers::HighLevelCommand();
  stateSub = lcm.subscribe(constants::ROBOT_STATE_TOPIC_NAME,
                           &LocalPlannerNode::stateHandle, this);
  stateSub->setQueueCapacity(1);

  commandSub = lcm.subscribe(constants::HIGH_LEVEL_COMMAND_TOPIC_NAME,
                             &LocalPlannerNode::commandHandle, this);
  commandSub->setQueueCapacity(1);
}

template <class RobotClass> bool LocalPlannerNode<RobotClass>::handle() {
  float commandDeltaTime =
      timePointDiffInSeconds(getWallTime(), lastCommandTimestamp);

  if (firstCommandRecieved && commandDeltaTime > COMMAND_TIMEOUT_SECONDS) {
    std::cout << "LocalPlannerNode: "
              << constants::HIGH_LEVEL_COMMAND_TOPIC_NAME
              << " topic timeout. Last message recieved " << commandDeltaTime
              << " seconds ago." << std::endl;
    return false;
  }
  return lcm.handle() == 0;
}

template <class RobotClass> void LocalPlannerNode<RobotClass>::processLoop() {
  while (handle())
    ;
}

template <class RobotClass>
LocalPlanner &LocalPlannerNode<RobotClass>::getLocalPlanner() {
  return localPlanner;
}

} // namespace control

template class control::LocalPlannerNode<robots::UnitreeA1>;
} // namespace strelka
