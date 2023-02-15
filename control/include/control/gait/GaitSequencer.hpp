#ifndef GAIT_SEQUENCER_H
#define GAIT_SEQUENCER_H

#include <algorithm>
#include <control/gait/Gait.hpp>
#include <control/gait/GaitSequencer.hpp>
#include <math.h>
#include <string.h>
#include <vector>

namespace strelka {
class GaitSequencer {
public:
  struct GaitSequenceElement {
    Gait gait;
    float duration;
  };

  LegState legState[4];
  float normalizedPhase[4];

private:
  std::vector<GaitSequenceElement> sequence;
  float currentGaitTime;
  int gaitIdx;

  void updateLegState();

public:
  GaitSequencer(Gait gait);

  GaitSequencer(const GaitSequencer &gait);

  void reset();

  void step(float dt);

  Gait &currentGait();
};
} // namespace strelka

#endif // GAIT_SEQUENCER_H