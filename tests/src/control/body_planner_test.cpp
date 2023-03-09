#include <gtest/gtest.h>
#include <strelka_control/BodyTrajectoryPlanner.hpp>
#include <strelka_messages/HighLevelCommand.hpp>
#include <strelka_robots/A1/UnitreeA1.hpp>
#include <strelka_robots/A1/constants.hpp>

namespace {

using namespace strelka::control;
using namespace strelka::robots;
using namespace strelka::messages;

TEST(BodyPlannerTests, TrajectoryIntegration) {
  float desiredVelocityX = 0.2;
  float dt = 0.001;
  int horizonSteps = 10;

  UnitreeA1 robot = UnitreeA1::createDummyA1RobotWithStateEstimates();
  BodyTrajectoryPlanner planner{};

  HighLevelCommand command =
      HighLevelCommand::makeDummyCommandMessage(desiredVelocityX, 0.0);

  DMat<float> trajectory =
      planner.getDesiredBodyTrajectory(robot, command, dt, horizonSteps);

  float trajectoryPosX = trajectory(horizonSteps - 1, 3);
  float integratedPosX = dt * desiredVelocityX * horizonSteps;
  EXPECT_NEAR(trajectoryPosX, integratedPosX, 1e-4);
}

} // namespace