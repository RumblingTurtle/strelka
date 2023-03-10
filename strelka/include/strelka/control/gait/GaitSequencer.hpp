#ifndef GAIT_SEQUENCER_H
#define GAIT_SEQUENCER_H

#include <algorithm>
#include <math.h>
#include <strelka/control/gait/Gait.hpp>
#include <strelka/control/gait/GaitSequencer.hpp>
#include <string.h>
#include <vector>

namespace strelka {
/**
 * @brief Class resposible for detemining current state of each leg during a
 * gait cycle.
 *
 * Every leg has a fixed alteration of swing and stance cycles in time.
 * Current relative time in current phase is equal to:
 *
 * tRel = (tAbs+phaseOffset*phaseDuration)%phaseDuration
 *
 * if the relative phase time is greater than the stance duration then the leg
 * is in swing phase and in stance otherwise.
 *
 * Absolute duration can be computed as phaseDuration*dutyFactor
 *
 * It is possible to use a sequence of gait and duration pairs.
 * For example [(TROT,3.0),(BOUND,2,0)]. Sequencer would run TROT for 3
 * seconds and BOUND for two seconds then switch back to TROT.
 *
 * Note that sequencer would not adjust gaits to provided durations. If you
 * specified a duration not divisible by the phase duration of the gait then
 * there is a possibility of having an sudden change in normalized phase
 * value.
 */
class GaitSequencer {

public:
  struct GaitSequenceElement {
    Gait gait;
    float duration; // In seconds
  };

  LegState legState[4];
  float normalizedPhase[4];

  GaitSequencer(Gait gait);

  GaitSequencer(const GaitSequencer &gait);

  void reset();

  void step(float dt);

  Gait &currentGait();

private:
  float currentGaitTime;

  int gaitIdx;

  std::vector<GaitSequenceElement> sequence;

  void updateLegState();
};
} // namespace strelka

#endif // GAIT_SEQUENCER_H