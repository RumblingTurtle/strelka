#ifndef BODY_TRAJECTORY_PLANNER_H
#define BODY_TRAJECTORY_PLANNER_H
#include <common/constants.hpp>
#include <common/rotation.hpp>
#include <common/typedefs.hpp>

namespace strelka {

class BodyTrajectoryPlanner {
  Mat34<float> prevContactPosWorld;
  Mat34<float> prevContactPosBody;
  bool firstRun;

public:
  struct BodyTrajectoryPlannerInput {
    Vec4<float> currentQuaternion;
    Vec3<float> currentPosition;
    Vec12<float> currentFootPositions;
    Vec4<bool> currentContacts;

    Vec3<float> desiredAngularVelocity;
    Vec3<float> desiredLinearVelocity;
    Vec3<float> desiredRPY;
    Vec3<float> comOffset;

    float desiredBodyHeight;

    float dt;
    int horizonSteps;
  };

  BodyTrajectoryPlanner();

  void getDesiredBodyTrajectory(BodyTrajectoryPlannerInput &input,
                                DMat<float> &trajectory);
};
} // namespace strelka

#endif // BODY_TRAJECTORY_PLANNER_H