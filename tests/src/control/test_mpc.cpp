#include <common/macros.hpp>
#include <control/BodyTrajectoryPlanner.hpp>
#include <control/FootholdPlanner.hpp>
#include <control/MPC.hpp>
#include <iostream>
#include <messages/HighLevelCommand.hpp>
#include <robots/A1/UnitreeA1.hpp>
#include <robots/A1/constants.hpp>

int main() {

  using namespace strelka;
  using namespace strelka::control;
  using namespace strelka::robots;
  using namespace strelka::messages;

  Gait TEST_GAIT = {.name = "test",
                    ._dutyFactor = {0.6, 0.6, 0.6, 0.6},
                    ._phaseDuration = {0.5, 0.5, 0.5, 0.5},
                    ._phaseOffset = {0.0, 0.0, 0.0, 0.0}};

  const float desiredVelocityX = 0.0;
  const float dt = 0.001;
  const int horizonSteps = 10;

  HighLevelCommand command =
      HighLevelCommand::makeDummyCommandMessage(desiredVelocityX);
  UnitreeA1 robot = createDummyA1RobotWithStateEstimates();

  GaitScheduler scheduler{TEST_GAIT};
  BodyTrajectoryPlanner bodyPlanner{};
  FootholdPlanner footPlanner{scheduler};

  MPC mpc(A1::constants::MPC_BODY_MASS, A1::constants::MPC_BODY_INERTIA,
          horizonSteps, dt);

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