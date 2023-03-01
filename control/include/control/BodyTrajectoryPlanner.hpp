#ifndef BODY_TRAJECTORY_PLANNER_H
#define BODY_TRAJECTORY_PLANNER_H
#include <common/Robot.hpp>
#include <common/constants.hpp>
#include <common/macros.hpp>
#include <common/rotation.hpp>
#include <common/typedefs.hpp>
#include <messages/HighLevelCommand.hpp>

namespace strelka {
namespace control {

class BodyTrajectoryPlanner {
  Mat43<float> prevContactPosWorld;
  Mat43<float> prevContactPosBody;

  bool firstRun;

public:
  BodyTrajectoryPlanner();

  DMat<float> getDesiredBodyTrajectory(robots::Robot &robot,
                                       messages::HighLevelCommand &command,
                                       float dt, int horizonSteps);

  DMat<float> getDesiredBodyTrajectoryTest(robots::Robot &robot,
                                           messages::HighLevelCommand &command,
                                           float dt, int horizonSteps);
};
} // namespace control
} // namespace strelka

#endif // BODY_TRAJECTORY_PLANNER_H