#include <gtest/gtest.h>
#include <strelka/common/macros.hpp>
#include <strelka/control/BodyTrajectoryPlanner.hpp>
#include <strelka/control/FootholdPlanner.hpp>
#include <strelka/robots/A1/UnitreeA1.hpp>
#include <strelka_messages/HighLevelCommand.hpp>

namespace {

using namespace strelka;
using namespace strelka::control;
using namespace strelka::robots;
using namespace strelka::messages;

class FootPlannerFixture : public ::testing::Test {
protected:
  const float desiredVelocityX = 0.2;
  const float dt = 0.001;
  const int horizonSteps = 10;

  Gait TEST_GAIT = {.name = "test",
                    ._dutyFactor = {0.6, 0.6, 0.6, 0.6},
                    ._phaseDuration = {0.5, 0.5, 0.5, 0.5},
                    ._phaseOffset = {0.0, 0.0, 0.0, 0.0},
                    ._stationaryGait = false};

  HighLevelCommand *command;

  std::unique_ptr<UnitreeA1> robot;
  std::shared_ptr<GaitScheduler> scheduler;
  std::shared_ptr<FootholdPlanner> footPlanner;

  void SetUp() override {
    command = new HighLevelCommand(
        HighLevelCommand::makeDummyCommandMessage(desiredVelocityX));
    robot = std::make_unique<UnitreeA1>(
        UnitreeA1::createDummyA1RobotWithStateEstimates());
    scheduler = std::make_shared<GaitScheduler>(TEST_GAIT);
    footPlanner = std::make_shared<FootholdPlanner>(scheduler, false);
  }

  void TearDown() override { delete command; }
};

TEST_F(FootPlannerFixture, FootPositionsOverHorizon) {

  DMat<bool> contactTable =
      scheduler->getContactTable(dt, horizonSteps, {1, 1, 1, 1});

  footPlanner->calculateNextFootholdPositions(*robot, *command);

  FOR_EACH_LEG {
    float footholdDist = (footPlanner->getFoothold(LEG_ID, 0) -
                          robot->footPositionWorldFrame(LEG_ID))
                             .norm();
    EXPECT_EQ(footholdDist, 0.0) << "First foothold was altered: " << LEG_ID;
  }
}
} // namespace