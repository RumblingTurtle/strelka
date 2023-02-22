#include <common/A1/constants.hpp>
#include <common/macros.hpp>
#include <control/BodyTrajectoryPlanner.hpp>
#include <control/FootholdPlanner.hpp>
#include <control/MPC.hpp>
#include <iostream>
#include <messages/HighLevelCommand.hpp>
#include <robots/UnitreeA1.hpp>

int main() {

  using namespace strelka;
  using namespace strelka::control;
  using namespace strelka::robots;
  using namespace strelka::messages;

  Gait TEST_GAIT = {.name = "test",
                    .dutyFactor = {0.6, 0.6, 0.6, 0.6},
                    .phaseDuration = {0.5, 0.5, 0.5, 0.5},
                    .phaseOffset = {0.0, 0.0, 0.0, 0.0}};

  const DVec<double> MPC_WEIGHTS =
      Eigen::Map<const DVec<double>>(constants::A1::MPC_WEIGHTS, 13);

  const float desiredVelocityX = 0.0;
  const float dt = 0.001;
  const int horizonSteps = 10;

  HighLevelCommand command =
      HighLevelCommand::makeDummyCommandMessage(desiredVelocityX);
  UnitreeA1 robot = createDummyA1RobotWithStateEstimates();

  GaitScheduler scheduler{TEST_GAIT};
  BodyTrajectoryPlanner bodyPlanner{};
  FootholdPlanner footPlanner{scheduler};

  MPC mpc(constants::A1::MPC_BODY_MASS, constants::A1::MPC_BODY_INERTIA,
          horizonSteps, dt, MPC_WEIGHTS, constants::A1::MPC_ALPHA,
          constants::A1::MPC_FRICTION_COEFFS,
          constants::A1::MPC_CONSTRAINT_MAX_SCALE,
          constants::A1::MPC_CONSTRAINT_MIN_SCALE);

  DMat<float> bodyTrajectory =
      bodyPlanner.getDesiredBodyTrajectory(robot, command, dt, horizonSteps);

  DMat<bool> contactTable =
      scheduler.getContactTable(dt, horizonSteps, {1, 1, 1, 1});

  DMat<float> footholdTable = footPlanner.calculateWorldFrameRotatedFootholds(
      robot, command, bodyTrajectory, contactTable);

  mpc.computeContactForces(robot, contactTable, footholdTable, bodyTrajectory);

  mpc.computeContactForces(robot, contactTable, footholdTable, bodyTrajectory);

  return 0;
}