#include <control/BodyTrajectoryPlanner.hpp>

namespace strelka {
namespace control {
BodyTrajectoryPlanner::BodyTrajectoryPlanner() : firstRun(true) {
  prevContactPosWorld.setZero();
  prevContactPosBody.setZero();
}
DMat<float> BodyTrajectoryPlanner::getDesiredBodyTrajectoryTest(
    robots::Robot &robot, messages::HighLevelCommand &command, float dt,
    int horizonSteps) {

  DMat<float> trajectory(horizonSteps, 13);
  trajectory.setZero();
  Vec3<float> desiredComOffset = command.desiredComOffset();
  Vec3<float> currentRPY = rotation::quat2euler(robot.bodyToWorldQuat());
  Mat3<float> bodyToWorldRot;
  Vec3<float> currentRobotPos = robot.positionWorldFrame();
  rotation::quat2rot(robot.bodyToWorldQuat(), bodyToWorldRot);

  for (int h = 0; h < horizonSteps; h++) {
    trajectory(h, 0) = 0;
    trajectory(h, 1) = 0;
    trajectory(h, 2) = currentRPY(2);

    trajectory(h, 3) = 0 + desiredComOffset(0);
    trajectory(h, 4) = 0 + desiredComOffset(1);
    trajectory(h, 5) = command.desiredBodyHeight();

    // Prefer to stablize roll and pitch.
    trajectory.block<1, 3>(h, 6).setZero();
    trajectory.block<1, 3>(h, 9).setZero();
    trajectory(h, 12) = constants::GRAVITY_CONSTANT;
  }

  firstRun = false;
  return trajectory;
}

DMat<float> BodyTrajectoryPlanner::getDesiredBodyTrajectory(
    robots::Robot &robot, messages::HighLevelCommand &command, float dt,
    int horizonSteps) {

  DMat<float> trajectory(horizonSteps, 13);
  trajectory.setZero();

  Vec3<float> currentRPY = rotation::quat2euler(robot.bodyToWorldQuat());
  Mat3<float> bodyToWorldRot;

  rotation::quat2rot(robot.bodyToWorldQuat(), bodyToWorldRot);

  FOR_EACH_LEG {
    if (robot.footContact(LEG_ID) || firstRun) {
      prevContactPosBody.block<1, 3>(LEG_ID, 0) =
          robot.footPositionTrunkFrame(LEG_ID).transpose();
      prevContactPosWorld.block<1, 3>(LEG_ID, 0) =
          robot.footPositionWorldFrame(LEG_ID).transpose();
    }
  }

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
