#include <control/gait/GaitSequencer.hpp>

namespace strelka {

void GaitSequencer::updateLegState() {
  Gait &gait = currentGait();

  for (int legId = 0; legId < 4; legId++) {
    float legTime =
        currentGaitTime + gait.phaseOffset(legId) * gait.phaseDuration(legId);

    float phaseTime = std::fmod(legTime, gait.phaseDuration(legId));

    float fullPhaseNormalized = phaseTime / gait.phaseDuration(legId);

    if (fullPhaseNormalized < gait.dutyFactor(legId)) {
      legState[legId] = LegState::STANCE;
      normalizedPhase[legId] = phaseTime / gait.stanceDuration(legId);
    } else {
      legState[legId] = LegState::SWING;

      normalizedPhase[legId] =
          (phaseTime - gait.stanceDuration(legId)) / gait.swingDuration(legId);
    }
  }
}

GaitSequencer::GaitSequencer(Gait gait) {
  float maxDuration =
      *std::max_element(gait._phaseDuration, gait._phaseDuration + 4);
  sequence.push_back({gait, 2 * maxDuration});
  reset();
}

GaitSequencer::GaitSequencer(const GaitSequencer &sequencer) {
  sequence = sequencer.sequence;
  memcpy(legState, sequencer.legState, sizeof(LegState) * 4);
  memcpy(normalizedPhase, sequencer.normalizedPhase, sizeof(float) * 4);
  currentGaitTime = sequencer.currentGaitTime;
  gaitIdx = sequencer.gaitIdx;
}

void GaitSequencer::reset() {
  for (int legId = 0; legId < 4; legId++) {
    legState[legId] = LegState::STANCE;
    normalizedPhase[legId] = 0;
  }
  currentGaitTime = 0;
  gaitIdx = 0;
}

void GaitSequencer::step(float dt) {
  assert(dt > 0);
  currentGaitTime += dt;

  if (currentGaitTime > sequence[gaitIdx].duration) {
    gaitIdx = (gaitIdx + 1) % sequence.size();
    currentGaitTime = 0;
  }

  updateLegState();
}

Gait &GaitSequencer::currentGait() { return sequence[gaitIdx].gait; }

} // namespace strelka
