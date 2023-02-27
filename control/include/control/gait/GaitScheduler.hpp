

#ifndef GAIT_SCHEDULER_H
#define GAIT_SCHEDULER_H

#include <common/typedefs.hpp>
#include <control/gait/GaitSequencer.hpp>

#define CONTACT_DETECTION_THRESHOLD 0.25

namespace strelka {
class GaitScheduler {
  float scale;
  GaitSequencer sequencer;

public:
  GaitScheduler(Gait gait);
  GaitScheduler(GaitSequencer sequencer);

  LegState currentLegState[4];
  LegState prevLegState[4];

  LegState scheduledState[4];
  LegState prevScheduledState[4];

  float _normalizedPhase[4];

  void setScale(float newScale);

  void reset();

  DMat<bool> getContactTable(float dt, int horizonSteps,
                             Vec4<bool> currentContacts);

  bool isLegSwinging(int legId);
  bool isLegScheduledToSwing(int legId);

  float normalizedPhase(int legId);
  float phaseDuration(int legId);
  float swingDuration(int legId);
  float stanceDuration(int legId);
  bool swingStarted(int legId);
  bool footInContact(int legId);
  bool lostContact(int legID);

  void step(float dt, Vec4<bool> contacts);
};
} // namespace strelka
#endif // GAIT_SCHEDULER_H