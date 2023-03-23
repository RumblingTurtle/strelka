#include <strelka/robots/A1/UnitreeA1.hpp>

namespace strelka {

namespace robots {

UnitreeA1::UnitreeA1() {}

UnitreeA1::UnitreeA1(const strelka_lcm_headers::RobotState *robotStateMessage) {
  initRawStateEntries(robotStateMessage);
  initStateEstimateEntries(robotStateMessage);
}

UnitreeA1::UnitreeA1(
    const strelka_lcm_headers::RobotRawState *rawStateMessage) {
  initRawStateEntries(rawStateMessage);
}

template <class MessageType>
void UnitreeA1::initRawStateEntries(const MessageType *message) {
  hasRawState = true;
  _bodyToWorldQuat =
      Eigen::Map<const Eigen::Matrix<float, 4, 1>>(message->quaternion, 4);

  rotation::quat2rot(_bodyToWorldQuat, _bodyToWorldMat);

  _q = Eigen::Map<const Vec12<float>>(message->q, 12);
  _dq = Eigen::Map<const Vec12<float>>(message->dq, 12);

  _gyroscopeBodyFrame = Eigen::Map<const Vec3<float>>(message->gyro, 3);
  _accelerometerBodyFrame = Eigen::Map<const Vec3<float>>(message->accel, 3);

  FOR_EACH_LEG {
    Vec3<float> trunkToHipOffset = Eigen::Map<const Vec3<float>>(
        A1::constants::TRUNK_TO_HIP_OFFSETS + 3 * LEG_ID, 3);

    _footJacobians.block<3, 3>(LEG_ID * 3, 0) =
        A1::kinematics::analyticalLegJacobian(_q.block<3, 1>(LEG_ID * 3, 0),
                                              LEG_ID);

    _footPositionsTrunkFrame.col(LEG_ID) =
        A1::kinematics::footPositionHipFrame(_q.block<3, 1>(LEG_ID * 3, 0),
                                             LEG_ID) +
        trunkToHipOffset;

    _footVelocitiesTrunkFrame.col(LEG_ID) =
        (_footJacobians.block<3, 3>(LEG_ID * 3, 0) *
         _dq.block<3, 1>(LEG_ID * 3, 0))
            .transpose();

    _footContacts(LEG_ID) =
        message->footForces[LEG_ID] > A1::constants::FOOT_FORCE_THRESHOLD;
  }

  _accelerometerWorldFrame =
      rotateBodyToWorldFrame(_accelerometerBodyFrame) +
      Vec3<float>{0.0f, 0.0f, constants::GRAVITY_CONSTANT};
}

void UnitreeA1::initStateEstimateEntries(
    const strelka_lcm_headers::RobotState *message) {
  hasStateEstimates = true;
  _positionWorldFrame = Eigen::Map<const Vec3<float>>(message->position, 3);
  _linearVelocityBodyFrame =
      Eigen::Map<const Vec3<float>>(message->velocityBody, 3);
}

Eigen::Matrix<float, 12, 3> UnitreeA1::footJacobians() {
  if (!hasRawState) {
    throw StatelessRobotException();
  }
  return _footJacobians;
}

Mat3<float> UnitreeA1::bodyToWorldMat() {
  if (!hasRawState) {
    throw StatelessRobotException();
  }
  return _bodyToWorldMat;
}

Vec3<float> UnitreeA1::rotateBodyToWorldFrame(Vec3<float> vector) {
  if (!hasRawState) {
    throw StatelessRobotException();
  }
  return _bodyToWorldMat * vector;
}

Vec3<float> UnitreeA1::rotateWorldToBodyFrame(Vec3<float> vector) {
  if (!hasRawState) {
    throw StatelessRobotException();
  }
  return _bodyToWorldMat.transpose() * vector;
}

Vec4<bool> UnitreeA1::footContacts() {
  if (!hasRawState) {
    throw StatelessRobotException();
  }
  return _footContacts;
}

Vec3<float> UnitreeA1::footPositionTrunkFrame(int legId) {
  if (!hasRawState) {
    throw StatelessRobotException();
  }
  return _footPositionsTrunkFrame.col(legId);
}

Vec3<float> UnitreeA1::footVelocityTrunkFrame(int legId) {
  if (!hasRawState) {
    throw StatelessRobotException();
  }
  return _footVelocitiesTrunkFrame.col(legId);
}
Vec3<float> UnitreeA1::gyroscopeBodyFrame() {
  if (!hasRawState) {
    throw StatelessRobotException();
  }
  return _gyroscopeBodyFrame;
}

Vec3<float> UnitreeA1::accelerometerWorldFrame() {
  if (!hasRawState) {
    throw StatelessRobotException();
  }
  return _accelerometerWorldFrame;
}

Vec12<float> UnitreeA1::q() {
  if (!hasRawState) {
    throw StatelessRobotException();
  }
  return _q;
};
Vec12<float> UnitreeA1::dq() {
  if (!hasRawState) {
    throw StatelessRobotException();
  }
  return _dq;
};

Quat<float> UnitreeA1::bodyToWorldQuat() {
  if (!hasRawState) {
    throw StatelessRobotException();
  }
  return _bodyToWorldQuat;
};

Vec3<float> UnitreeA1::bodyToWorldRPY() {
  return rotation::quat2euler(bodyToWorldQuat());
}

Vec3<float> UnitreeA1::footPositionWorldFrame(int legId) {
  return rotateBodyToWorldFrame(footPositionTrunkFrame(legId)) +
         positionWorldFrame();
}

Vec3<float> UnitreeA1::transformBodyToWorldFrame(Vec3<float> vector) {
  return rotateBodyToWorldFrame(vector) + positionWorldFrame();
}

Vec3<float> UnitreeA1::transformWorldToBodyFrame(Vec3<float> vector) {
  return rotateWorldToBodyFrame(vector - positionWorldFrame());
}

Vec3<float> UnitreeA1::positionWorldFrame() {
  if (!hasStateEstimates) {
    throw NoStateEstimateException();
  }
  return _positionWorldFrame;
};

Vec3<float> UnitreeA1::linearVelocityBodyFrame() {
  if (!hasStateEstimates) {
    throw NoStateEstimateException();
  }
  return _linearVelocityBodyFrame;
};

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

Vec3<float> UnitreeA1::trunkToThighOffset(int legId) {
  return Eigen::Map<const Vec3<float>>(
      A1::constants::TRUNK_TO_THIGH_OFFSETS + legId * 3, 3);
}

Vec3<float> UnitreeA1::bodyToComOffset() { return A1::constants::COM_OFFSET; }

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