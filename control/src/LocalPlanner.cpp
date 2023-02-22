#include <control/LocalPlanner.hpp>

namespace strelka {
namespace control {

LocalPlanner::LocalPlanner()
    : scheduler(GAITS::STAND), bodyPlanner(), footPlanner(scheduler),
      prevTick(-1) {
  const DVec<double> MPC_WEIGHTS =
      Eigen::Map<const DVec<double>>(constants::A1::MPC_WEIGHTS, 13);

  mpc = new MPC(constants::A1::MPC_BODY_MASS, constants::A1::MPC_BODY_INERTIA,
                horizonSteps, MPC_DT, MPC_WEIGHTS, constants::A1::MPC_ALPHA,
                constants::A1::MPC_FRICTION_COEFFS,
                constants::A1::MPC_CONSTRAINT_MAX_SCALE,
                constants::A1::MPC_CONSTRAINT_MIN_SCALE);

  const std::vector<double> inertia{0.15, 0, 0, 0, 0.34, 0, 0, 0, 0.36};
  const std::vector<double> weights{constants::A1::MPC_WEIGHTS,
                                    constants::A1::MPC_WEIGHTS + 13};
  oldMpc = new Mpc(constants::A1::MPC_BODY_MASS, inertia, 4, horizonSteps,
                   MPC_DT, weights, 1e-5);

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
      messages::HighLevelCommand::makeDummyCommandMessage(0.0);
  robots::UnitreeA1 robot{messageIn};

  if (prevTick == -1) {
    prevTick = messageIn->tick - 0.001;
  }

  float dt = messageIn->tick - prevTick;
  Vec4<bool> footContacts = robot.footContacts();
  scheduler.step(dt, footContacts);

  DMat<bool> contactTable =
      scheduler.getContactTable(MPC_DT, horizonSteps, footContacts);

  DMat<float> bodyTrajectory = bodyPlanner.getDesiredBodyTrajectoryTest(
      robot, command, MPC_DT, horizonSteps);

  footPlanner.calculateNextFootholdPositions(robot, command);

  DMat<float> footholdTable =
      footPlanner.calculateWorldFrameRotatedFootholdsTest(
          robot, command, bodyTrajectory, contactTable);

  if (false) {
    std::vector<double> coeffs{0.45, 0.45, 0.45, 0.45};

    DVec<double> currentState(13);
    currentState.block(0, 0, 3, 1) = robot.currentRPY().cast<double>();
    currentState.block(3, 0, 3, 1) = robot.positionWorldFrame().cast<double>();

    currentState.block(6, 0, 3, 1) =
        robot.rotateBodyToWorldFrame(robot.gyroscopeBodyFrame()).cast<double>();

    currentState.block(9, 0, 3, 1) =
        robot.rotateBodyToWorldFrame(robot.linearVelocityBodyFrame())
            .cast<double>();

    currentState(12) = (double)constants::GRAVITY_CONSTANT;

    DMat<double> contcts = contactTable.cast<double>();
    DMat<double> feet = footholdTable.cast<double>();
    DMat<double> b = bodyTrajectory.cast<double>();
    DVec<double> body = Eigen::Map<DVec<double>>(b.data(), b.size());
    std::vector<double> forces =
        oldMpc->ComputeContactForces(contcts, feet, body, currentState, coeffs,
                                     constants::A1::MPC_CONSTRAINT_MAX_SCALE,
                                     constants::A1::MPC_CONSTRAINT_MIN_SCALE);
    FOR_EACH_LEG {
      wbicCommand->mpcForces[LEG_ID * 3] = -forces[LEG_ID * 3];
      wbicCommand->mpcForces[LEG_ID * 3 + 1] = -forces[LEG_ID * 3 + 1];
      wbicCommand->mpcForces[LEG_ID * 3 + 2] = -forces[LEG_ID * 3 + 2];
    }
  } else {
    DVec<double> forces = mpc->computeContactForces(
        robot, contactTable, footholdTable, bodyTrajectory);

    Vec12<float> mpcForces = -forces.block(0, 0, 12, 1).cast<float>();

    FOR_EACH_LEG {
      wbicCommand->mpcForces[LEG_ID * 3] = mpcForces[LEG_ID * 3];
      wbicCommand->mpcForces[LEG_ID * 3 + 1] = mpcForces[LEG_ID * 3 + 1];
      wbicCommand->mpcForces[LEG_ID * 3 + 2] = mpcForces[LEG_ID * 3 + 2];
    }
  }

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
  /*
  commandSub =
      lcm.subscribe("high_command", &LocalPlanner::commandHandle, this);
  commandSub->setQueueCapacity(1);
  */
  stateSub->setQueueCapacity(1);
  while (lcm.handle() == 0) {
  }
}
} // namespace control
} // namespace strelka
