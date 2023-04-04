#include <strelka/control/gait/GaitScheduler.hpp>

namespace strelka {
GaitScheduler::GaitScheduler(Gait gait) : sequencer(gait) { reset(); }

GaitScheduler::GaitScheduler(GaitSequencer sequencer) : sequencer(sequencer) {
  reset();
}

void GaitScheduler::setScale(float newScale) {
  assert(newScale > 0);
  scale = newScale;
}

void GaitScheduler::reset() {
  scale = 1;
  sequencer.reset();
  for (int legId = 0; legId < 4; legId++) {
    currentLegState[legId] = LegState::STANCE;
    prevLegState[legId] = LegState::STANCE;
    scheduledState[legId] = LegState::STANCE;
    prevScheduledState[legId] = LegState::STANCE;
  }
}

DMat<bool> GaitScheduler::getContactTable(float dt, int horizonSteps) {
  // TODO: assert matrix dims to 4xhorizonSteps
  DMat<bool> contactTable(4, horizonSteps);
  contactTable.setZero();

  GaitSequencer sequencerCopy = this->sequencer;

  for (int h = 0; h < horizonSteps; h++) {
    for (int legId = 0; legId < 4; legId++) {
      contactTable(legId, h) = sequencerCopy.legState[legId] != LegState::SWING;
    }
    sequencerCopy.step(dt * scale);
  }
  return contactTable;
}

void GaitScheduler::step(float dt, Vec4<bool> contacts) {
  memcpy(prevScheduledState, scheduledState, sizeof(LegState) * 4);
  memcpy(prevLegState, currentLegState, sizeof(LegState) * 4);

  sequencer.step(dt * scale);

  memcpy(scheduledState, sequencer.legState, sizeof(LegState) * 4);
  memcpy(currentLegState, sequencer.legState, sizeof(LegState) * 4);

  for (int legId = 0; legId < 4; legId++) {
    // Keep the leg in early contact until the swing phase stops
    if (scheduledState[legId] == LegState::SWING &&
        prevLegState[legId] == LegState::EARLY_CONTACT) {
      currentLegState[legId] = LegState::EARLY_CONTACT;
    }

    if (scheduledState[legId] == LegState::SWING && contacts[legId] &&
        normalizedPhase(legId) >= CONTACT_DETECTION_THRESHOLD) {
      currentLegState[legId] = LegState::EARLY_CONTACT;
    }

    if (scheduledState[legId] == LegState::STANCE && !contacts[legId]) {
      currentLegState[legId] = LegState::LOST_CONTACT;
    }
  }
}

bool GaitScheduler::isLegScheduledToSwing(int legId) {
  return scheduledState[legId] == LegState::SWING;
}

bool GaitScheduler::swingStarted(int legId) {
  return scheduledState[legId] == LegState::SWING &&
         prevScheduledState[legId] == LegState::STANCE;
}

bool GaitScheduler::legInSwing(int legId) {
  return currentLegState[legId] == LegState::SWING;
}

bool GaitScheduler::legInEarlyContact(int legId) {
  return currentLegState[legId] == LegState::EARLY_CONTACT;
}

bool GaitScheduler::legInStance(int legId) {
  return currentLegState[legId] == LegState::STANCE;
}

bool GaitScheduler::legLostContact(int legId) {
  return currentLegState[legId] == LegState::LOST_CONTACT;
}

float GaitScheduler::GaitScheduler::normalizedPhase(int legId) {
  return sequencer.normalizedPhase[legId];
}

float GaitScheduler::phaseDuration(int legId) {
  return sequencer.currentGait().phaseDuration(legId);
}

float GaitScheduler::swingDuration(int legId) {
  return sequencer.currentGait().swingDuration(legId);
}

float GaitScheduler::stanceDuration(int legId) {
  return sequencer.currentGait().stanceDuration(legId);
}

bool GaitScheduler::isCurrentGaitStationary() {
  return sequencer.currentGait().isStationary();
}
}; // namespace strelka