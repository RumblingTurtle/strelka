#ifndef FOOTHOLD_PLANNER_H
#define FOOTHOLD_PLANNER_H

#include <common/A1/constants.hpp>
#include <common/constants.hpp>
#include <common/macros.hpp>
#include <common/rotation.hpp>
#include <common/trajectory.hpp>
#include <common/typedefs.hpp>
#include <control/gait/GaitScheduler.hpp>
#include <exception>
#include <messages/HighLevelCommand.hpp>
#include <robots/Robot.hpp>
#include <vector>

#define FOOT_CLEARANCE 0.02
#define FEEDBACK_GAIN 0.1

namespace strelka {
namespace control {

class InvalidFootholdIdx {
  const char *what() {
    return "FootholdPlanner: invalid foothold index in getFoothold";
  }
};

class FootholdPlanner {
  GaitScheduler &gaitScheduler;

  std::vector<std::vector<Vec3<float>>> _footholds;

  float swingHeight[4];
  bool swingBack[4];

  Mat43<float> lastContactPosWorld;

  bool firstRun;
  bool updateContinuously;

public:
  enum class FOOTHOLD_PREDICTION_TYPE { SIMPLE, RAIBERT, CONTINUOUS };

  FootholdPlanner(GaitScheduler &gaitScheduler);

  int footholdCount(int legId);

  Vec3<float> getFoothold(int legId, int footholdId);

  DMat<float> calculateBodyFrameFootholds(robots::Robot &robot,
                                          messages::HighLevelCommand &command,
                                          DMat<float> &bodyTrajectory,
                                          DMat<bool> &contactTable);

  void calculateNextFootholdPositions(robots::Robot &robot,
                                      messages::HighLevelCommand &command);
  Vec3<float> predictNextFootPos(
      Vec3<float> currentPosition, Mat3<float> bodyToWorldRot,
      Vec3<float> prevFootPosition, Vec3<float> desiredLinearVelocity,
      Vec3<float> desiredAngularVelocity, Vec3<float> bodyLinearVelocity,
      float feedbackGain, int legId, FOOTHOLD_PREDICTION_TYPE predictType);

  void getFootDesiredPVA(robots::Robot &robot,
                         Vec12<float> &desiredFootPositions,
                         Vec12<float> &desiredFootVelocities,
                         Vec12<float> &desiredFootAccelerations);
};

} // namespace control
} // namespace strelka

#endif // FOOTHOLD_PLANNER_H