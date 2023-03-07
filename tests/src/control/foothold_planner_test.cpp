#include <common/macros.hpp>
#include <control/BodyTrajectoryPlanner.hpp>
#include <control/FootholdPlanner.hpp>
#include <gtest/gtest.h>
#include <messages/HighLevelCommand.hpp>
#include <robots/A1/UnitreeA1.hpp>
#include <robots/A1/constants.hpp>

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

  UnitreeA1 *robot;
  GaitScheduler *scheduler;
  FootholdPlanner *footPlanner;

  void SetUp() override {
    robot = new UnitreeA1(UnitreeA1::createDummyA1RobotWithStateEstimates());
    command = new HighLevelCommand(
        HighLevelCommand::makeDummyCommandMessage(desiredVelocityX));
    scheduler = new GaitScheduler(TEST_GAIT);
    footPlanner = new FootholdPlanner(*scheduler);
  }

  void TearDown() override {
    delete scheduler;
    delete footPlanner;
    delete robot;
    delete command;
  }
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