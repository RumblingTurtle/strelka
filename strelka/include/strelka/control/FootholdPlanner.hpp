#ifndef FOOTHOLD_PLANNER_H
#define FOOTHOLD_PLANNER_H

#include <exception>
#include <memory>
#include <strelka/common/constants.hpp>
#include <strelka/common/macros.hpp>
#include <strelka/common/rotation.hpp>
#include <strelka/common/trajectory.hpp>
#include <strelka/common/typedefs.hpp>
#include <strelka/control/gait/GaitScheduler.hpp>
#include <strelka/robots/Robot.hpp>
#include <strelka_messages/HighLevelCommand.hpp>
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
  std::shared_ptr<GaitScheduler> _gaitScheduler;
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
  bool firstRun;
  bool updateContinuously;

  Mat34<float> lastContactPosWorld;
  Mat34<float> _currentFootPosition;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::shared_ptr<GaitScheduler> &gaitScheduler() { return _gaitScheduler; };
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

  FootholdPlanner(std::shared_ptr<GaitScheduler> _gaitScheduler);
  FootholdPlanner(FootholdPlanner &footholdPlanner);

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

  /**
   * @brief Updates footholds for each leg.
   *
   * @param robot Object which implements Robot interface
   * @param command HighLevelCommand object
   */
  void calculateNextFootholdPositions(robots::Robot &robot,
                                      messages::HighLevelCommand &command);
  /**
   * @brief Returns foot world in odometry frame after
   * calculateWorldFrameRotatedFootholds update
   *
   * @param LEG_ID leg id
   * @return Vec3<float> foot position in world frame
   */
  Vec3<float> currentFootPosition(int legId);

  Vec3<float> predictNextFootPos(const Vec3<float> &currentPosition,
                                 const Mat3<float> &bodyToWorldRot,
                                 const Vec3<float> &prevFootPosition,
                                 robots::Robot &robot,
                                 messages::HighLevelCommand &command,
                                 float feedbackGain, int legId,
                                 FOOTHOLD_PREDICTION_TYPE predictType);

  /**
   * @brief Get desired foot position velocity and acceleration in odometry
   * frame. Return current position of the foot if the legState is STANCE,
   * LOSE_CONTACT or EARLY_CONTACT.
   *
   * @param robot Object implementing Robot interface
   * @param command  HighLevelCommand object
   * @param desiredFootPositions
   * @param desiredFootVelocities
   * @param desiredFootAccelerations
   */
  void getFootDesiredPVA(robots::Robot &robot,
                         messages::HighLevelCommand &command,
                         Vec12<float> &desiredFootPositions,
                         Vec12<float> &desiredFootVelocities,
                         Vec12<float> &desiredFootAccelerations);

  /**
   * @brief Get the desired trajectory positon for visualization.
   * Swing height, duration and swingBack parameters are used from the first
   * foothold pair
   *
   * @param pStart Start position
   * @param pEnd  End position
   * @param t Swing normal phase value
   * @return Vec3<float> Bezier curve position in pStart and pEnd coordinate
   * frame
   */
  Vec3<float> getDesiredTrajectoryPosition(const Vec3<float> &pStart,
                                           const Vec3<float> &pEnd, float t,
                                           int legId);
  /**
   * @brief Adjusts foothold according to some criterion
   *
   * Default implementation assumes that the next foothold's height is on
   * the same level as the starting position.
   *
   * @returns Vec3<float> Adjusted foothold position in world frame
   */
  virtual Vec3<float> adjustFoothold(const Vec3<float> &nominalFootPosition,
                                     const Vec3<float> &currentRobotPosition,
                                     const Mat3<float> &currentRobotRotation,
                                     int legId, robots::Robot &robot);
};

} // namespace control
} // namespace strelka

#endif // FOOTHOLD_PLANNER_H