#include <control/BodyTrajectoryPlanner.hpp>

namespace strelka {

BodyTrajectoryPlanner::BodyTrajectoryPlanner() : firstRun(true) {
  prevContactPosWorld.setZero();
  prevContactPosBody.setZero();
}

void BodyTrajectoryPlanner::getDesiredBodyTrajectory(
    BodyTrajectoryPlannerInput &input, DMat<float> &trajectory) {

  Vec3<float> currentRPY;
  Mat3<float> bodyToWorldRot;

  rotation::quat2euler(input.currentQuaternion, currentRPY);
  rotation::quat2rot(input.currentQuaternion, bodyToWorldRot);

  for (int legId = 0; legId < 4; legId++) {
    if (input.currentContacts[legId] || firstRun) {
      Vec3<float> currentFootPosBody = Eigen::Map<const Vec3<float>>(
          input.currentFootPositions.data() + legId * 3, 3);
      prevContactPosBody.block(legId, 0, 1, 3) = currentFootPosBody.transpose();
      prevContactPosWorld.block(legId, 0, 1, 3) =
          (bodyToWorldRot * currentFootPosBody + input.currentPosition)
              .transpose();
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

  for (int h = 0; h < input.horizonSteps; h++) {
    trajectory(h, 0) = input.desiredRPY(0);
    trajectory(h, 1) = estimatedTerrainPitch;
    trajectory(h, 2) =
        currentRPY(2) + input.dt * (h + 1) * input.desiredAngularVelocity(2);

    // Desired rotation and linear velocity of the body at horizon step h
    Mat3<float> rotation_h;
    Vec3<float> velocity_h;
    Vec3<float> rpy_h = trajectory.block(h, 0, 1, 3);
    rotation::rpy2rot(rpy_h, rotation_h);
    velocity_h = rotation_h * input.desiredLinearVelocity;

    if (h == 0) {
      Vec3<float> comOffsetWorld = rotation_h * input.comOffset;
      trajectory(h, 3) = input.currentPosition(0) + comOffsetWorld(0);
      trajectory(h, 4) = input.currentPosition[1] + comOffsetWorld(1);
      trajectory(h, 5) =
          input.desiredBodyHeight + estimatedContactHeight + comOffsetWorld(2);
    } else {
      trajectory.block(h, 3, 1, 3) = trajectory.block(h - 1, 3, 1, 3);
    }

    trajectory.block(h, 3, 1, 3) += input.dt * velocity_h;

    // Prefer to stablize roll and pitch.
    trajectory.block(h, 6, 1, 3) = input.desiredAngularVelocity;
    trajectory.block(h, 9, 1, 3) = velocity_h;
    trajectory(h, 12) = constants::GRAVITY_CONSTANT(2);
  }

  firstRun = false;
}
} // namespace strelka
