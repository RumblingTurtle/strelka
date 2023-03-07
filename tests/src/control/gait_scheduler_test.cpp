
#include <common/macros.hpp>
#include <control/gait/Gait.hpp>
#include <control/gait/GaitScheduler.hpp>
#include <gtest/gtest.h>

namespace {
using namespace strelka;

class GaitSchedulerFixture : public ::testing::Test {
protected:
  Gait TEST_GAIT;
  GaitScheduler *scheduler;

  void SetUp() override {
    TEST_GAIT = {.name = "test",
                 ._dutyFactor = {0.6, 0.6, 0.6, 0.6},
                 ._phaseDuration = {0.5, 0.5, 0.5, 0.5},
                 ._phaseOffset = {0.0, 0.0, 0.0, 0.0}};

    scheduler = new GaitScheduler{TEST_GAIT};
  }

  void TearDown() override { delete scheduler; }
};

TEST_F(GaitSchedulerFixture, Contacts) {
  scheduler->step(0.4, {1, 1, 1, 1});

  FOR_EACH_LEG {
    EXPECT_TRUE(scheduler->legInStance(LEG_ID)) << "LEG_ID: " << LEG_ID;
  }
}

TEST_F(GaitSchedulerFixture, PhaseParameters) {

  scheduler->step(0.4, {1, 1, 1, 1});

  FOR_EACH_LEG {
    EXPECT_EQ(scheduler->phaseDuration(LEG_ID), TEST_GAIT.phaseDuration(LEG_ID))
        << "LEG_ID: " << LEG_ID;

    EXPECT_EQ(scheduler->swingDuration(LEG_ID),
              (1 - TEST_GAIT.dutyFactor(LEG_ID)) *
                  TEST_GAIT.phaseDuration(LEG_ID))
        << "LEG_ID: " << LEG_ID;

    EXPECT_EQ(scheduler->stanceDuration(LEG_ID),
              TEST_GAIT.dutyFactor(LEG_ID) * TEST_GAIT.phaseDuration(LEG_ID))
        << "LEG_ID: " << LEG_ID;
  }
}

TEST_F(GaitSchedulerFixture, NormalPhase) {

  scheduler->step(0.4, {1, 1, 1, 1});

  FOR_EACH_LEG {
    EXPECT_EQ(scheduler->normalizedPhase(LEG_ID), 0.5) << "LEG_ID: " << LEG_ID;
  }
}

TEST_F(GaitSchedulerFixture, FootStates) {

  scheduler->step(0.4, {1, 1, 1, 1});

  FOR_EACH_LEG {
    EXPECT_TRUE(scheduler->isLegScheduledToSwing(LEG_ID))
        << "LEG_ID: " << LEG_ID;

    EXPECT_TRUE(scheduler->swingStarted(LEG_ID)) << "LEG_ID: " << LEG_ID;

    EXPECT_TRUE(!scheduler->legLostContact(LEG_ID)) << "LEG_ID: " << LEG_ID;
  }
}

} // namespace
