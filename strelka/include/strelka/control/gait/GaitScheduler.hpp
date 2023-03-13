

#ifndef GAIT_SCHEDULER_H
#define GAIT_SCHEDULER_H

#include <strelka/common/typedefs.hpp>
#include <strelka/control/gait/GaitSequencer.hpp>

#define CONTACT_DETECTION_THRESHOLD 0.25

namespace strelka {
/**
 * @brief Contact aware gait sequencer with more complex queries regarding
 * current leg state. Gaits can also be sped up and slowed down.
 *
 */
class GaitScheduler {

  float scale;
  GaitSequencer sequencer;

public:
  LegState currentLegState[4];
  LegState prevLegState[4];

  LegState scheduledState[4];
  LegState prevScheduledState[4];

  GaitScheduler(Gait gait);
  GaitScheduler(GaitSequencer sequencer);

  GaitScheduler(const GaitScheduler &gait) = delete;
  GaitScheduler(GaitScheduler &gait) = delete;

  /**
   * @brief Sets a scale for the next supplied step dt's provided to step(...)
   * method
   */
  void setScale(float newScale);

  void reset();

  /**
   * @brief Return boolean 4xhorizonSteps matrix representing expected contact
   * schedule of the legs by making horizonSteps steps dt seconds each
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

  bool isLegScheduledToSwing(int legId);

  float normalizedPhase(int legId);

  float phaseDuration(int legId);

  float swingDuration(int legId);

  float stanceDuration(int legId);

  bool swingStarted(int legId);

  bool legInSwing(int legId);

  bool legInEarlyContact(int legId);

  bool legInStance(int legId);

  bool legLostContact(int legId);

  bool isCurrentGaitStationary();

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