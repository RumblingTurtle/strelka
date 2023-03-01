#include <robots/A1/control/A1LocalPlanner.hpp>

namespace strelka {
namespace control {

A1LocalPlanner::A1LocalPlanner()
    : prevTick(-1), localPlanner(A1::constants::MPC_BODY_MASS,
                                 A1::constants::MPC_BODY_INERTIA) {
  wbicCommand = new a1_lcm_msgs::WbicCommand();
}

A1LocalPlanner::~A1LocalPlanner() {
  delete wbicCommand;
  lcm.unsubscribe(stateSub);
  lcm.unsubscribe(commandSub);
}

void A1LocalPlanner::stateHandle(const lcm::ReceiveBuffer *rbuf,
                                 const std::string &chan,
                                 const a1_lcm_msgs::RobotState *messageIn) {

  messages::HighLevelCommand command =
      messages::HighLevelCommand::makeDummyCommandMessage(0.1, 0.2);
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
  memcpy(wbicCommand->pFoot, localPlanner.desiredFootP().data(),
         sizeof(float) * 12);
  memcpy(wbicCommand->vFoot, localPlanner.desiredFootV().data(),
         sizeof(float) * 12);
  memcpy(wbicCommand->aFoot, localPlanner.desiredFootA().data(),
         sizeof(float) * 12);

  wbicCommand->stop = 0;

  lcm.publish("wbic_command", wbicCommand);
}

void A1LocalPlanner::commandHandle(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const a1_lcm_msgs::HighLevelCommand *commandMsg) {
  memcpy(highCommand, commandMsg, sizeof(a1_lcm_msgs::HighLevelCommand));
}

void A1LocalPlanner::processLoop() {
  stateSub = lcm.subscribe("robot_state", &A1LocalPlanner::stateHandle, this);
  /*
  commandSub =
      lcm.subscribe("high_command", &LocalPlanner::commandHandle, this);
  commandSub->setQueueCapacity(1);
  */
  stateSub->setQueueCapacity(1);
  while (lcm.handle() == 0)
    ;
}
} // namespace control
} // namespace strelka
