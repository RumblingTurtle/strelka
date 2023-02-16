#ifndef BODY_TRAJECTORY_PLANNER_H
#define BODY_TRAJECTORY_PLANNER_H
#include <common/constants.hpp>
#include <common/macros.hpp>
#include <common/rotation.hpp>
#include <common/typedefs.hpp>
#include <messages/HighLevelCommand.hpp>
#include <robots/Robot.hpp>

namespace strelka {

class BodyTrajectoryPlanner {
  Mat34<float> prevContactPosWorld;
  Mat34<float> prevContactPosBody;
  bool firstRun;
  float dt;
  int horizonSteps;

public:
  BodyTrajectoryPlanner(float dt, int horizonSteps);

  void getDesiredBodyTrajectory(robots::Robot &robot,
                                messages::HighLevelCommand &command,
                                DMat<float> &trajectory);
};
} // namespace strelka

#endif // BODY_TRAJECTORY_PLANNER_H