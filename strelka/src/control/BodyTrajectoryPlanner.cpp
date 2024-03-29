#include <strelka/control/BodyTrajectoryPlanner.hpp>

namespace strelka {
namespace control {
BodyTrajectoryPlanner::BodyTrajectoryPlanner(float heightFilterCutoffFrequency,
                                             float pitchFilterCutoffFrequency)
    // 30.0 10.0 stairs
    // 0.9 0.9 sparse stepping stones
    : firstRun(true), heightFilter(0.001, heightFilterCutoffFrequency),
      pitchFilter(0.001, pitchFilterCutoffFrequency) {
  prevContactPosWorld.setZero();
  prevContactPosBody.setZero();
}

DMat<float> BodyTrajectoryPlanner::getDesiredBodyTrajectory(
    robots::Robot &robot, messages::HighLevelCommand &command, float dt,
    int horizonSteps) {

  DMat<float> trajectory(horizonSteps, 13);
  trajectory.setZero();

  Vec3<float> bodyToWorldRPY = robot.bodyToWorldRPY();
  Mat3<float> bodyToWorldRot;

  rotation::quat2rot(robot.bodyToWorldQuat(), bodyToWorldRot);
  Vec4<bool> footContacts = robot.footContacts();

  FOR_EACH_LEG {
    if (footContacts(LEG_ID) || firstRun) {
      prevContactPosBody.col(LEG_ID) = robot.footPositionTrunkFrame(LEG_ID);
      prevContactPosWorld.col(LEG_ID) = robot.footPositionWorldFrame(LEG_ID);
    }
  }

  // Fit least squares through last contact points
  Eigen::Matrix<float, 4, 3> A;
  A.setOnes();
  A.block<4, 2>(0, 0) = prevContactPosBody.block<2, 4>(0, 0).transpose();

  Vec4<float> b = prevContactPosBody.block<1, 4>(2, 0).transpose();
  Vec3<float> slopeCoefficients = A.colPivHouseholderQr().solve(b);

  float slopePitch = -slopeCoefficients(0);
  slopePitch = clamp<float>(slopePitch, -M_PI_2 * 0.6, M_PI_2 * 0.6);
  float estimatedTerrainPitch = pitchFilter.filter(slopePitch);
  float estimatedContactHeight =
      heightFilter.filter(prevContactPosWorld.row(2).mean());

  Vec3<float> desiredAngularVelocity =
      command.desiredAngularVelocityBodyFrame();
  Vec3<float> desiredLinearVelocity = command.desiredLinearVelocityBodyFrame();

  for (int h = 0; h < horizonSteps; h++) {
    trajectory(h, 0) = command.desiredRPY()(0);
    trajectory(h, 1) = estimatedTerrainPitch;
    trajectory(h, 2) =
        bodyToWorldRPY(2) + dt * (h + 1) * desiredAngularVelocity(2);

    Mat3<float> rotation_h;
    Vec3<float> rpy_h = trajectory.block<1, 3>(h, 0).transpose();
    rotation::rpy2rot(rpy_h, rotation_h);
    Vec3<float> velocity_h = rotation_h * desiredLinearVelocity;
    velocity_h(2) = 0;

    if (h == 0) {
      Vec3<float> comOffsetWorld = rotation_h * command.desiredComOffset();
      Vec3<float> currentRobotPos = robot.positionWorldFrame();

      trajectory(h, 3) = currentRobotPos(0) + comOffsetWorld(0);
      trajectory(h, 4) = currentRobotPos(1) + comOffsetWorld(1);
      trajectory(h, 5) = command.desiredBodyHeight() + estimatedContactHeight +
                         comOffsetWorld(2);
    } else {
      trajectory.block<1, 3>(h, 3) = trajectory.block<1, 3>(h - 1, 3);
    }

    trajectory.block<1, 3>(h, 3) += (dt * velocity_h).transpose();

    // Prefer to stablize roll and pitch.
    trajectory.block<1, 3>(h, 6) = desiredAngularVelocity.transpose();
    trajectory.block<1, 3>(h, 9) = velocity_h.transpose();
    trajectory(h, 12) = constants::GRAVITY_CONSTANT;
  }

  firstRun = false;
  return trajectory;
}
} // namespace control
} // namespace strelka
