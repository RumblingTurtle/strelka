#ifndef FOOTHOLD_PLANNER_H
#define FOOTHOLD_PLANNER_H

#include <common/Robot.hpp>
#include <common/constants.hpp>
#include <common/macros.hpp>
#include <common/rotation.hpp>
#include <common/trajectory.hpp>
#include <common/typedefs.hpp>
#include <control/gait/GaitScheduler.hpp>
#include <exception>
#include <messages/HighLevelCommand.hpp>
#include <vector>

#define FOOT_CLEARANCE 0.02
#define FEEDBACK_GAIN 0.1

namespace strelka {
namespace control {

class InvalidFootholdIdx {
public:
  const char *what() {
    return "FootholdPlanner: invalid foothold index in getFoothold";
  }
};

/**
 * @brief Class responsible for determining next foothold positions in the world
 * and body frames
 *
 *
 */
class FootholdPlanner {
  GaitScheduler &gaitScheduler;
  /**
   * @brief World frame footholds for each leg.
   *
   * Usually each leg has 3-4 entries but no less than 2.
   * The first one is always the starting position of the leg's swing phase.
   * Second is the swing's end position. The rest are added depending on how
   * long is the horizon provided to calculateWorldFrameRotatedFootholds for
   * performance reasons.
   *
   */
  std::vector<Vec3<float>> _footholds[4];

  float swingHeight[4];
  bool swingBack[4];

  Mat43<float> lastContactPosWorld;

  bool firstRun;
  bool updateContinuously;

public:
  /**
   * @brief Nominal foothold prediction types
   * Simple:
   * previous foot position + phase duration * desired velocity
   *
   * Raibert:
   * hip position + ( swing duration + 0.5*stance duration ) * desired velocity
   *
   * Continuous:
   * hip position + ( swing time left + 0.5*stance duration ) * desired velocity
   *
   */
  enum class FOOTHOLD_PREDICTION_TYPE { SIMPLE, RAIBERT, CONTINUOUS };

  FootholdPlanner(GaitScheduler &gaitScheduler);

  /**
   * @brief Get the amount of predicted footholds+1
   *
   * @param legId leg index
   * @return int the amount of predicted footholds+1
   */
  int footholdCount(int legId);

  /**
   * @brief Get the foothold for given foot at a given index
   *
   * @param legId leg index
   * @param footholdId foothold index
   * @return Vec3<float> foothold position in the world frame
   */
  Vec3<float> getFoothold(int legId, int footholdId);

  /**
   * @brief Given body trajectory and foot contact table for the next N steps
   * generates foothold positions in the world frame at each horizon step minus
   * body position at that step
   *
   * @param robot Object which implements Robot interface
   * @param command HighLevelCommand object
   *
   * @param bodyTrajectory Table of size Nx13 which contains body trajectory.
   * See BodyTrajectoryPlanner for an example
   *
   * @param contactTable Table of size 4xN with contact/no contact pattern per
   * each leg. See GaitScheduler's getContactTable for details
   *
   * @return DMat<float> Table of size 4x3*N with each triplet in a row
   * representing foothold position in world frame at step n minus body position
   * at that step
   */
  DMat<float> calculateWorldFrameRotatedFootholds(
      robots::Robot &robot, messages::HighLevelCommand &command,
      DMat<float> &bodyTrajectory, DMat<bool> &contactTable);

  DMat<float> calculateWorldFrameRotatedFootholdsTest(
      robots::Robot &robot, messages::HighLevelCommand &command,
      DMat<float> &bodyTrajectory, DMat<bool> &contactTable);

  /**
   * @brief Updates footholds for each leg.
   *
   * @param robot Object which implements Robot interface
   * @param command HighLevelCommand object
   */
  void calculateNextFootholdPositions(robots::Robot &robot,
                                      messages::HighLevelCommand &command);

  Vec3<float>
  predictNextFootPos(Vec3<float> currentPosition, Mat3<float> bodyToWorldRot,
                     Vec3<float> prevFootPosition, robots::Robot &robot,
                     messages::HighLevelCommand &command, float feedbackGain,
                     int legId, FOOTHOLD_PREDICTION_TYPE predictType);

  /**
   * @brief Get desired foot position velocity and acceleration in odometry
   * frame. Return current position of the foot if the legState is STANCE,
   * LOSE_CONTACT or EARLY_CONTACT.
   *
   * @param robot Object implementing Robot interface
   * @param desiredFootPositions
   * @param desiredFootVelocities
   * @param desiredFootAccelerations
   */
  void getFootDesiredPVA(robots::Robot &robot,
                         Vec12<float> &desiredFootPositions,
                         Vec12<float> &desiredFootVelocities,
                         Vec12<float> &desiredFootAccelerations);
};

} // namespace control
} // namespace strelka

#endif // FOOTHOLD_PLANNER_H