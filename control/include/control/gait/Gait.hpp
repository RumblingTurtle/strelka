#ifndef GAIT_H
#define GAIT_H
#include <cassert>

namespace strelka {

enum class LegState { SWING, STANCE, EARLY_CONTACT, LOST_CONTACT };

struct Gait {
  /*
  Gait parameters container
  */
  const char *name;
  float dutyFactor[4];
  float phaseDuration[4];
  float phaseOffset[4];

  inline float swingDuration(int legId) {
    assert(legId >= 0 && legId < 4);
    return (1 - dutyFactor[legId]) * phaseDuration[legId];
  }

  inline float stanceDuration(int legId) {
    assert(legId >= 0 && legId < 4);
    return dutyFactor[legId] * phaseDuration[legId];
  }
};

namespace GAITS {
static Gait STAND = {.name = "stand",
                     .dutyFactor = {1, 1, 1, 1},
                     .phaseDuration = {0.3, 0.3, 0.3, 0.3},
                     .phaseOffset = {0, 0, 0, 0}};

static Gait STEP = {.name = "step",
                    .dutyFactor = {0.2, 1, 1, 1},
                    .phaseDuration = {10, 10, 10, 10},
                    .phaseOffset = {0, 0, 0, 0}};

static Gait TROT = {.name = "trot",
                    .dutyFactor = {0.6, 0.6, 0.6, 0.6},
                    .phaseDuration = {0.5, 0.5, 0.5, 0.5},
                    .phaseOffset = {0.0, 0.5, 0.5, 0.0}};

static Gait FLYTROT = {.name = "flytrot",
                       .dutyFactor = {0.4, 0.4, 0.4, 0.4},
                       .phaseDuration = {0.3, 0.3, 0.3, 0.3},
                       .phaseOffset = {0.0, 0.5, 0.5, 0.0}};

static Gait GALOP = {.name = "galop",
                     .dutyFactor = {0.2, 0.2, 0.2, 0.2},
                     .phaseDuration = {0.5, 0.5, 0.5, 0.5},
                     .phaseOffset = {0.0, 0.8571, 0.3571, 0.5}};

static Gait BOUND = {.name = "bound",
                     .dutyFactor = {0.3, 0.3, 0.3, 0.3},
                     .phaseDuration = {0.3, 0.3, 0.3, 0.3},
                     .phaseOffset = {0.0, 0.0, 0.414, 0.414}};

static Gait PRONK = {.name = "pronk",
                     .dutyFactor = {0.3, 0.3, 0.3, 0.3},
                     .phaseDuration = {0.5, 0.5, 0.5, 0.5},
                     .phaseOffset = {0, 0, 0, 0}};
} // namespace GAITS

} // namespace strelka

#endif // GAIT_H