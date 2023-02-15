#include <control/gait/GaitScheduler.hpp>

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
  for (int legId = 0; legId < 4; legId++) {
    currentLegState[legId] = LegState::STANCE;
    prevLegState[legId] = LegState::STANCE;
    scheduledState[legId] = LegState::STANCE;
    prevScheduledState[legId] = LegState::STANCE;
    _normalizedPhase[legId] = 0;
  }
}

void GaitScheduler::getContactTable(float dt, int horizonSteps,
                                    Vec4<bool> currentContacts,
                                    DMat<bool> &contactTable) {
  // TODO: assert matrix dims to 4xhorizonSteps
  GaitSequencer sequencerCopy = this->sequencer;
  LegState lastState[4];
  Vec4<bool> earlyContact = currentContacts;

  for (int h = 0; h < horizonSteps; h++) {
    for (int legId = 0; legId < 4; legId++) {
      LegState currentState = sequencerCopy.legState[legId];

      if (h == 0) {
        lastState[legId] = currentState;
      }

      bool swingStarted = lastState[legId] == LegState::STANCE and
                          currentState == LegState::SWING;

      if (swingStarted) {
        earlyContact[legId] = false;
      }

      if (earlyContact[legId]) {
        contactTable(legId, h) = currentContacts[legId];
      } else {
        contactTable(legId, h) = currentState == LegState::STANCE;
      }

      lastState[legId] = currentState;
    }
    sequencerCopy.step(dt * scale);
  }
}

void GaitScheduler::step(float dt, Vec4<bool> contacts) {
  memcpy(prevScheduledState, scheduledState, sizeof(LegState) * 4);
  memcpy(prevLegState, currentLegState, sizeof(LegState) * 4);

  sequencer.step(dt * scale);

  memcpy(_normalizedPhase, sequencer.normalizedPhase, sizeof(float) * 4);
  memcpy(scheduledState, sequencer.legState, sizeof(LegState) * 4);
  memcpy(currentLegState, sequencer.legState, sizeof(LegState) * 4);

  for (int legId = 0; legId < 4; legId++) {

    if (scheduledState[legId] == LegState::SWING && contacts[legId] &&
        _normalizedPhase[legId] > CONTACT_DETECTION_THRESHOLD)
      currentLegState[legId] = LegState::EARLY_CONTACT;

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

bool GaitScheduler::footInContact(int legId) {
  return currentLegState[legId] == LegState::EARLY_CONTACT ||
         currentLegState[legId] == LegState::STANCE;
}

bool GaitScheduler::lostContact(int legId) {
  return currentLegState[legId] == LegState::LOST_CONTACT;
}

float GaitScheduler::GaitScheduler::normalizedPhase(int legId) {
  return _normalizedPhase[legId];
}

float GaitScheduler::phaseDuration(int legId) {
  return sequencer.currentGait().phaseDuration[legId];
}

float GaitScheduler::swingDuration(int legId) {
  return sequencer.currentGait().swingDuration(legId);
}

float GaitScheduler::stanceDuration(int legId) {
  return sequencer.currentGait().stanceDuration(legId);
}

}; // namespace strelka