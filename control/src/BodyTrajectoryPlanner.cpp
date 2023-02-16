#include <control/BodyTrajectoryPlanner.hpp>

namespace strelka {
namespace control {
BodyTrajectoryPlanner::BodyTrajectoryPlanner() : firstRun(true) {
  prevContactPosWorld.setZero();
  prevContactPosBody.setZero();
}

DMat<float> BodyTrajectoryPlanner::getDesiredBodyTrajectory(
    robots::Robot &robot, messages::HighLevelCommand &command, float dt,
    int horizonSteps) {

  DMat<float> trajectory(horizonSteps, 12);
  trajectory.setZero();

  Vec3<float> currentRPY;
  Mat3<float> bodyToWorldRot;

  rotation::quat2euler(robot.bodyToWorldQuat(), currentRPY);
  rotation::quat2rot(robot.bodyToWorldQuat(), bodyToWorldRot);

  FOR_EACH_LEG {
    if (robot.footContact(LEG_ID) || firstRun) {
      prevContactPosBody.block(LEG_ID, 0, 1, 3) =
          robot.footPositionTrunkFrame(LEG_ID).transpose();
      prevContactPosWorld.block(LEG_ID, 0, 1, 3) =
          robot.footPositionWorldFrame(LEG_ID).transpose();
    }
  }

  // Fit least squares through last contact points
  /*        A = np.hstack(
      (prevContactPosBody[:, :2], np.array([[1, 1, 1, 1]]).T))
  b = prevContactPosBody[:, 2].T

  coeffs = A.solve(B)

  if (firstRun) {
      pitch_filter.filter(0);
      height_filter.filter(estimated_height);
  }

      estimatedTerrainPitch = pitch_filter.filter(-coeffs[0])
      estimatedHeight = height_filter.filter(estimated_height)
  */

  float estimatedTerrainPitch = 0;
  float estimatedContactHeight = prevContactPosWorld.col(2).mean();

  Vec3<float> desiredAngularVelocity =
      command.desiredAngularVelocityBodyFrame();
  Vec3<float> desiredLinearVelocity = command.desiredLinearVelocityBodyFrame();

  for (int h = 0; h < horizonSteps; h++) {
    trajectory(h, 0) = command.desiredRPY()(0);
    trajectory(h, 1) = estimatedTerrainPitch;
    trajectory(h, 2) = currentRPY(2) + dt * (h + 1) * desiredAngularVelocity(2);

    // Desired rotation and linear velocity of the body at horizon step h
    Mat3<float> rotation_h;
    Vec3<float> rpy_h = trajectory.block(h, 0, 1, 3).transpose();
    rotation::rpy2rot(rpy_h, rotation_h);
    Vec3<float> velocity_h = rotation_h * desiredLinearVelocity;

    if (h == 0) {
      Vec3<float> comOffsetWorld = rotation_h * command.desiredComOffset();
      Vec3<float> currentRobotPos = robot.positionWorldFrame();

      trajectory(h, 3) = currentRobotPos(0) + comOffsetWorld(0);
      trajectory(h, 4) = currentRobotPos(1) + comOffsetWorld(1);
      trajectory(h, 5) = command.desiredBodyHeight() + estimatedContactHeight +
                         comOffsetWorld(2);
    } else {
      trajectory.block(h, 3, 1, 3) = trajectory.block(h - 1, 3, 1, 3);
    }

    trajectory.block(h, 3, 1, 3) += (dt * velocity_h).transpose();

    // Prefer to stablize roll and pitch.
    trajectory.block(h, 6, 1, 3) = desiredAngularVelocity.transpose();
    trajectory.block(h, 9, 1, 3) = velocity_h.transpose();
    trajectory(h, 11) = constants::GRAVITY_CONSTANT(2);
  }

  firstRun = false;
  return trajectory;
}
} // namespace control
} // namespace strelka
