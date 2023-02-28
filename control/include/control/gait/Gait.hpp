/**
 * @file Gait.hpp
 * Simple gait parameters container with definitions of common gait types and
 * leg states.
 * Leg IDs are specified in a given order:
 *
 * 0 Front right (FR)
 * 1 Front left (FL)
 * 2 Rear right (RR)
 * 3 Rear left (RL)
 */
#ifndef GAIT_H
#define GAIT_H
#include <cassert>

namespace strelka {

enum class LegState { SWING, STANCE, EARLY_CONTACT, LOST_CONTACT };

struct Gait {
  const char *name;
  float _dutyFactor[4];
  float _phaseDuration[4];
  float _phaseOffset[4];

  inline float phaseOffset(int legId) {
    assert(legId >= 0 && legId < 4);
    return _phaseOffset[legId];
  }

  inline float dutyFactor(int legId) {
    assert(legId >= 0 && legId < 4);
    return _dutyFactor[legId];
  }

  inline float phaseDuration(int legId) {
    assert(legId >= 0 && legId < 4);
    return _phaseDuration[legId];
  }

  inline float swingDuration(int legId) {
    assert(legId >= 0 && legId < 4);
    return (1 - dutyFactor(legId)) * phaseDuration(legId);
  }

  inline float stanceDuration(int legId) {
    assert(legId >= 0 && legId < 4);
    return dutyFactor(legId) * phaseDuration(legId);
  }
};

namespace GAITS {
static Gait STAND = {.name = "stand",
                     ._dutyFactor = {1, 1, 1, 1},
                     ._phaseDuration = {0.3, 0.3, 0.3, 0.3},
                     ._phaseOffset = {0, 0, 0, 0}};

static Gait STEP = {.name = "step",
                    ._dutyFactor = {0.2, 1, 1, 1},
                    ._phaseDuration = {10, 10, 10, 10},
                    ._phaseOffset = {0, 0, 0, 0}};

static Gait TROT = {.name = "trot",
                    ._dutyFactor = {0.6, 0.6, 0.6, 0.6},
                    ._phaseDuration = {0.5, 0.5, 0.5, 0.5},
                    ._phaseOffset = {0.0, 0.5, 0.5, 0.0}};

static Gait FLYTROT = {.name = "flytrot",
                       ._dutyFactor = {0.4, 0.4, 0.4, 0.4},
                       ._phaseDuration = {0.3, 0.3, 0.3, 0.3},
                       ._phaseOffset = {0.0, 0.5, 0.5, 0.0}};

static Gait GALOP = {.name = "galop",
                     ._dutyFactor = {0.2, 0.2, 0.2, 0.2},
                     ._phaseDuration = {0.5, 0.5, 0.5, 0.5},
                     ._phaseOffset = {0.0, 0.8571, 0.3571, 0.5}};

static Gait BOUND = {.name = "bound",
                     ._dutyFactor = {0.3, 0.3, 0.3, 0.3},
                     ._phaseDuration = {0.3, 0.3, 0.3, 0.3},
                     ._phaseOffset = {0.0, 0.0, 0.414, 0.414}};

static Gait PRONK = {.name = "pronk",
                     ._dutyFactor = {0.3, 0.3, 0.3, 0.3},
                     ._phaseDuration = {0.5, 0.5, 0.5, 0.5},
                     ._phaseOffset = {0, 0, 0, 0}};
} // namespace GAITS

} // namespace strelka

#endif // GAIT_H