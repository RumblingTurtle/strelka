#include <common/macros.hpp>
#include <control/BodyTrajectoryPlanner.hpp>
#include <control/FootholdPlanner.hpp>
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

  const float desiredVelocityX = 0.2;
  const float dt = 0.001;
  const int horizonSteps = 10;

  UnitreeA1 robot = UnitreeA1::createDummyA1RobotWithStateEstimates();
  HighLevelCommand command =
      HighLevelCommand::makeDummyCommandMessage(desiredVelocityX);
  GaitScheduler scheduler{TEST_GAIT};
  BodyTrajectoryPlanner bodyPlanner{};
  FootholdPlanner footPlanner{scheduler};

  DMat<float> bodyTrajectory =
      bodyPlanner.getDesiredBodyTrajectory(robot, command, dt, horizonSteps);

  DMat<bool> contactTable =
      scheduler.getContactTable(dt, horizonSteps, {1, 1, 1, 1});

  footPlanner.calculateNextFootholdPositions(robot, command);

  FOR_EACH_LEG {
    float footholdDist = (footPlanner.getFoothold(LEG_ID, 0) -
                          robot.footPositionWorldFrame(LEG_ID))
                             .norm();
    assert(footholdDist == 0.0);
  }

  DMat<float> footholdTable = footPlanner.calculateWorldFrameRotatedFootholds(
      robot, command, bodyTrajectory, contactTable);

  std::cout << footholdTable.row(0) << std::endl;

  Vec12<float> p;
  Vec12<float> v;
  Vec12<float> a;

  for (int i = 0; i < 100; i++) {
    footPlanner.getFootDesiredPVA(robot, p, v, a);
  }

  return 0;
}