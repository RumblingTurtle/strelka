
#include <common/macros.hpp>
#include <control/gait/Gait.hpp>
#include <control/gait/GaitScheduler.hpp>
#include <iostream>

int main() {
  using namespace strelka;
  Gait TEST_GAIT = {.name = "test",
                    ._dutyFactor = {0.6, 0.6, 0.6, 0.6},
                    ._phaseDuration = {0.5, 0.5, 0.5, 0.5},
                    ._phaseOffset = {0.0, 0.0, 0.0, 0.0}};

  GaitScheduler scheduler{TEST_GAIT};

  scheduler.step(0.4, {1, 1, 1, 1});

  assert(APPROX_EQUAL(scheduler.phaseDuration(0), TEST_GAIT.phaseDuration(0)));
  assert(
      APPROX_EQUAL(scheduler.swingDuration(0),
                   (1 - TEST_GAIT.dutyFactor(0)) * TEST_GAIT.phaseDuration(0)));

  assert(APPROX_EQUAL(scheduler.stanceDuration(0),
                      TEST_GAIT.dutyFactor(0) * TEST_GAIT.phaseDuration(0)));

  assert(APPROX_EQUAL(scheduler.normalizedPhase(0), 0.5));
  assert(scheduler.isLegScheduledToSwing(0));
  assert(scheduler.swingStarted(0));
  assert(scheduler.footInContact(0));
  assert(!scheduler.lostContact(0));

  return 0;
}