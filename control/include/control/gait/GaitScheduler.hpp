

#ifndef GAIT_SCHEDULER_H
#define GAIT_SCHEDULER_H

#include <common/typedefs.hpp>
#include <control/gait/GaitSequencer.hpp>

#define CONTACT_DETECTION_THRESHOLD 0.25

namespace strelka {
class GaitScheduler {
  /**
   * @brief Contact aware gait sequencer with more complex queries regarding
   * current leg state. Gaits can also be sped up and slowed down.
   *
   */
  float scale;
  GaitSequencer sequencer;

public:
  LegState currentLegState[4];
  LegState prevLegState[4];

  LegState scheduledState[4];
  LegState prevScheduledState[4];

  GaitScheduler(Gait gait);
  GaitScheduler(GaitSequencer sequencer);

  /**
   * @brief Sets a scale for the next supplied step dt's provided to step(...)
   * method
   */
  void setScale(float newScale);

  void reset();

  /**
   * @brief Return boolean 4xhorizonSteps matrix representing expected contact
   * schedule of the legs by stepping forward horizonSteps*dt seconds further in
   * time
   *
   * If an entry in currentContacts is set as true then the boolean values for a
   * leg would remain true until the next swing phase occurs.
   *
   * @param dt Time per step
   * @param horizonSteps Step count
   * @param currentContacts
   * @return DMat<bool> Expected contact table
   */
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

  /**
   * @brief Update the leg states given elapsed time since last update and
   * current contact state of the legs
   *
   * @param dt
   * @param contacts
   */
  void step(float dt, Vec4<bool> contacts);
};
} // namespace strelka
#endif // GAIT_SCHEDULER_H