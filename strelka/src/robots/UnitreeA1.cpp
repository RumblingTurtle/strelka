#include <strelka/robots/A1/UnitreeA1.hpp>
namespace strelka {

namespace robots {

UnitreeA1::UnitreeA1() {}

UnitreeA1::UnitreeA1(const strelka_lcm_headers::RobotState *robotStateMessage) {
  update(robotStateMessage);
}

UnitreeA1::UnitreeA1(
    const strelka_lcm_headers::RobotRawState *rawStateMessage) {
  update(rawStateMessage);
}

Mat3<float> UnitreeA1::footJacobianImpl(int legId) {
  return A1::kinematics::analyticalLegJacobian(_q.block<3, 1>(legId * 3, 0),
                                               legId);
}

Vec3<float> UnitreeA1::footPositionTrunkFrameImpl(int legId) {
  return A1::kinematics::footPositionHipFrame(_q.block<3, 1>(legId * 3, 0),
                                              legId) +
         trunkToHipOffset(legId);
}

bool UnitreeA1::estimateContactImpl(int legId) {
  return _footForces(legId) > A1::constants::FOOT_FORCE_THRESHOLD;
}

bool UnitreeA1::worldFrameIKCheck(Vec3<float> footPositionWorldFrame,
                                  int legId) {
  Vec3<float> trunkFrameFootPosition =
      rotateWorldToBodyFrame(footPositionWorldFrame - positionWorldFrame());
  return A1::kinematics::trunkFrameIKCheck(trunkFrameFootPosition, legId);
}

UnitreeA1 &UnitreeA1::createDummyA1RobotWithRawState(Vec3<float> motorAngles) {
  strelka_lcm_headers::RobotRawState dummyState{
      .quaternion = {1, 0, 0, 0},
      .gyro = {0, 0, 0},
      .accel = {0, 0, 0},
      .footForces = {30, 30, 30, 30},
      .position = {0, 0, 0},
      .velocity = {0, 0, 0},
      .angularVelocity = {0, 0, 0},
      .dq = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      .tick = 0.0,
  };

  memcpy(dummyState.q, motorAngles.data(), sizeof(float) * 12);
  static UnitreeA1 robot(&dummyState);
  return robot;
}

UnitreeA1 &UnitreeA1::createDummyA1RobotWithStateEstimates() {
  strelka_lcm_headers::RobotState dummyState{
      .quaternion = {1, 0, 0, 0},
      .gyro = {0, 0, 0},
      .accel = {0, 0, 0},
      .footForces = {30, 30, 30, 30},
      .dq = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      .tick = 0.0,
      .position = {0, 0, 0.25},
      .velocityBody = {0, 0, 0}};

  for (int motorId = 0; motorId < 12; motorId++) {
    bool rightLeg = motorId % 6 == 0;
    dummyState.q[motorId] =
        -A1::constants::STAND_ANGLES[motorId % 3] * rightLeg +
        A1::constants::STAND_ANGLES[motorId % 3] * (1 - rightLeg);
  }
  static UnitreeA1 robot(&dummyState);
  return robot;
}

Vec3<float> UnitreeA1::trunkToHipOffset(int legId) {
  return Eigen::Map<const Vec3<float>>(
      A1::constants::TRUNK_TO_HIP_OFFSETS + legId * 3, 3);
}

Vec3<float> UnitreeA1::trunkToThighOffset(int legId) {
  return trunkToHipOffset(legId) +
         Eigen::Map<const Vec3<float>>(
             A1::constants::HIP_TO_THIGH_OFFSET + legId * 3, 3);
}

Vec3<float> UnitreeA1::bodyToComOffset() {
  return A1::constants::TRUNK_TO_COM_OFFSET;
}

Vec3<float> UnitreeA1::bodyDimensions() {
  return A1::constants::BODY_DIMENSIONS;
}

Vec3<float> UnitreeA1::legDimensions() { return A1::constants::LEG_LENGTH; }

Mat3<float> UnitreeA1::rotationalInertia() {
  return A1::constants::BODY_INERTIA_MATRIX;
}

const float UnitreeA1::footRadius() { return A1::constants::FOOT_RADIUS; }
const float UnitreeA1::trunkMass() { return A1::constants::TRUNK_MASS; }
const float UnitreeA1::robotMass() { return A1::constants::TRUNK_MASS; }

Vec3<float> UnitreeA1::positionGains() { return A1::constants::POSITION_GAINS; }
Vec3<float> UnitreeA1::dampingGains() { return A1::constants::DAMPING_GAINS; }
Vec3<float> UnitreeA1::initAngles() { return A1::constants::INIT_ANGLES; };
Vec3<float> UnitreeA1::standAngles() { return A1::constants::STAND_ANGLES; };

} // namespace robots
} // namespace strelka