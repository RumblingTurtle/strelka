#include <control/LocalPlanner.hpp>

namespace strelka {
namespace control {

LocalPlanner::LocalPlanner()
    : scheduler(GAITS::TROT), bodyPlanner(), footPlanner(scheduler),
      prevTick(-1) {
  const DVec<double> MPC_WEIGHTS =
      Eigen::Map<const DVec<double>>(constants::A1::MPC_WEIGHTS, 13);

  mpc = new MPC(constants::A1::MPC_BODY_MASS, constants::A1::MPC_BODY_INERTIA,
                horizonSteps, MPC_DT, MPC_WEIGHTS, constants::A1::MPC_ALPHA,
                constants::A1::MPC_FRICTION_COEFFS,
                constants::A1::MPC_CONSTRAINT_MAX_SCALE,
                constants::A1::MPC_CONSTRAINT_MIN_SCALE);
  wbicCommand = new a1_lcm_msgs::WbicCommand();
}

LocalPlanner::~LocalPlanner() {
  delete mpc;
  delete wbicCommand;
  lcm.unsubscribe(stateSub);
  lcm.unsubscribe(commandSub);
}

void LocalPlanner::stateHandle(const lcm::ReceiveBuffer *rbuf,
                               const std::string &chan,
                               const a1_lcm_msgs::RobotState *messageIn) {

  messages::HighLevelCommand command =
      messages::HighLevelCommand::makeDummyCommandMessage(0.2);
  robots::UnitreeA1 robot{messageIn};

  if (prevTick == -1) {
    prevTick = messageIn->tick - 0.001;
  }

  float dt = messageIn->tick - prevTick;

  scheduler.step(dt, robot.footContacts().cast<bool>());

  DMat<float> bodyTrajectory =
      bodyPlanner.getDesiredBodyTrajectory(robot, command, dt, horizonSteps);

  DMat<bool> contactTable =
      scheduler.getContactTable(MPC_DT, horizonSteps, {1, 1, 1, 1});

  DMat<float> footholdTable = footPlanner.calculateBodyFrameFootholds(
      robot, command, bodyTrajectory, contactTable);

  DVec<double> forces = mpc->computeContactForces(
      robot, contactTable, footholdTable, bodyTrajectory);

  Vec12<float> mpcForces = forces.block(0, 0, 12, 1).cast<float>();

  Vec12<float> footP;
  Vec12<float> footV;
  Vec12<float> footA;

  footPlanner.getFootDesiredPVA(robot, footP, footV, footA);

  for (int i = 0; i < 3; i++) {
    wbicCommand->rpy[i] = bodyTrajectory(0, 0 + i);
    wbicCommand->pBody[i] = bodyTrajectory(0, 3 + i);
    wbicCommand->angularVelocity[i] = bodyTrajectory(0, 6 + i);
    wbicCommand->vBody[i] = bodyTrajectory(0, 9 + i);
    wbicCommand->aBody[i] = 0;
  }

  float footState[4];
  FOR_EACH_LEG {
    wbicCommand->footState[LEG_ID] =
        scheduler.footInContact(LEG_ID) || scheduler.lostContact(LEG_ID);
  }

  memcpy(wbicCommand->pFoot, footP.data(), sizeof(float) * 12);
  memcpy(wbicCommand->vFoot, footA.data(), sizeof(float) * 12);
  memcpy(wbicCommand->aFoot, footV.data(), sizeof(float) * 12);
  memcpy(wbicCommand->mpcForces, mpcForces.data(), sizeof(float) * 12);

  wbicCommand->stop = 0;

  lcm.publish("wbic_command", wbicCommand);
}

void LocalPlanner::commandHandle(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const a1_lcm_msgs::HighLevelCommand *commandMsg) {
  memcpy(highCommand, commandMsg, sizeof(a1_lcm_msgs::HighLevelCommand));
}

void LocalPlanner::processLoop() {
  stateSub = lcm.subscribe("robot_state", &LocalPlanner::stateHandle, this);
  commandSub =
      lcm.subscribe("high_command", &LocalPlanner::commandHandle, this);

  while (lcm.handle() == 0) {
  }
}
} // namespace control
} // namespace strelka
