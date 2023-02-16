#include <common/A1/constants.hpp>
#include <common/macros.hpp>
#include <control/BodyTrajectoryPlanner.hpp>
#include <control/FootholdPlanner.hpp>
#include <iostream>
#include <messages/HighLevelCommand.hpp>
#include <robots/UnitreeA1.hpp>

int main() {

  using namespace strelka;
  using namespace strelka::control;
  using namespace strelka::robots;
  using namespace strelka::messages;

  UnitreeA1 robot = createDummyA1RobotWithStateEstimates();

  Gait TEST_GAIT = {.name = "test",
                    .dutyFactor = {0.6, 0.6, 0.6, 0.6},
                    .phaseDuration = {0.5, 0.5, 0.5, 0.5},
                    .phaseOffset = {0.0, 0.0, 0.0, 0.0}};

  GaitScheduler scheduler{TEST_GAIT};

  BodyTrajectoryPlanner bodyPlanner{};
  float desiredVelocityX = 0.2;
  float dt = 0.001;
  int horizonSteps = 10;

  a1_lcm_msgs::HighLevelCommand highCommandMsg{
      .linearSpeed = {desiredVelocityX, 0, 0},
      .angularVelocity = {0, 0, 0},
      .footHeight = 0.12,
      .footClearance = 0.02,
      .hipOffsets = {0, 0},
      .rpy = {0, 0, 0},
      .comOffset = {0, 0},
      .bodyHeight = 0.25,
      .stop = false};

  HighLevelCommand command{&highCommandMsg};

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

  DMat<float> footholdTable = footPlanner.calculateBodyFrameFootholds(
      robot, command, bodyTrajectory, contactTable);

  std::cout << footholdTable.row(0) << std::endl;
  return 0;
}