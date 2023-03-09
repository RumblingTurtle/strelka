#include <strelka_robots/A1/UnitreeA1.hpp>

namespace strelka {

namespace robots {

UnitreeA1::~UnitreeA1(){};

UnitreeA1::UnitreeA1(const a1_lcm_msgs::RobotState *robotStateMessage)
    : hasStateEstimates(true) {
  initRawStateEntries(robotStateMessage);
  initStateEstimateEntries(robotStateMessage);
}

UnitreeA1::UnitreeA1(const a1_lcm_msgs::RobotRawState *rawStateMessage)
    : hasStateEstimates(false) {
  initRawStateEntries(rawStateMessage);
}

template <class MessageType>
void UnitreeA1::initRawStateEntries(const MessageType *message) {
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

    _footPositionsTrunkFrame.row(LEG_ID) =
        A1::kinematics::footPositionHipFrame(_q.block<3, 1>(LEG_ID * 3, 0),
                                             LEG_ID) +
        trunkToHipOffset;

    _footVelocitiesTrunkFrame.row(LEG_ID) =
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
    const a1_lcm_msgs::RobotState *message) {
  _positionWorldFrame = Eigen::Map<const Vec3<float>>(message->position, 3);
  _linearVelocityBodyFrame =
      Eigen::Map<const Vec3<float>>(message->velocityBody, 3);
}

Eigen::Matrix<float, 12, 3> UnitreeA1::footJacobians() {
  return _footJacobians;
}

Mat3<float> UnitreeA1::bodyToWorldMat() { return _bodyToWorldMat; }

Vec3<float> UnitreeA1::rotateBodyToWorldFrame(Vec3<float> vector) {
  return _bodyToWorldMat * vector;
}

Vec3<float> UnitreeA1::rotateWorldToBodyFrame(Vec3<float> vector) {
  return _bodyToWorldMat.transpose() * vector;
}

Vec3<float> UnitreeA1::transformBodyToWorldFrame(Vec3<float> vector) {
  return rotateBodyToWorldFrame(vector) + positionWorldFrame();
}

Vec3<float> UnitreeA1::transformWorldToBodyFrame(Vec3<float> vector) {
  return rotateWorldToBodyFrame(vector - positionWorldFrame());
}

bool UnitreeA1::footContact(int legId) { return _footContacts(legId); }

Vec4<bool> UnitreeA1::footContacts() { return _footContacts; }

Vec3<float> UnitreeA1::footPositionTrunkFrame(int legId) {
  return _footPositionsTrunkFrame.row(legId);
}

Vec3<float> UnitreeA1::footVelocityTrunkFrame(int legId) {
  return _footVelocitiesTrunkFrame.row(legId);
}

Vec3<float> UnitreeA1::footPositionWorldFrame(int legId) {
  return rotateBodyToWorldFrame(footPositionTrunkFrame(legId)) +
         positionWorldFrame();
}

Vec3<float> UnitreeA1::gyroscopeBodyFrame() { return _gyroscopeBodyFrame; }

Vec3<float> UnitreeA1::accelerometerWorldFrame() {
  return _accelerometerWorldFrame;
}

Vec12<float> UnitreeA1::q() { return _q; };
Vec12<float> UnitreeA1::dq() { return _dq; };

Vec3<float> UnitreeA1::positionWorldFrame() {
  if (hasStateEstimates) {
    return _positionWorldFrame;
  } else {
    throw NoStateEstimateException();
  }
};

Vec3<float> UnitreeA1::linearVelocityBodyFrame() {
  if (hasStateEstimates) {
    return _linearVelocityBodyFrame;
  } else {
    throw NoStateEstimateException();
  }
};

Quat<float> UnitreeA1::bodyToWorldQuat() { return _bodyToWorldQuat; };

Vec3<float> UnitreeA1::currentRPY() {
  return rotation::quat2euler(bodyToWorldQuat());
}

Vec3<float> UnitreeA1::trunkToThighOffset(int legId) {
  return Eigen::Map<const Vec3<float>>(
      A1::constants::TRUNK_TO_THIGH_OFFSETS + legId * 3, 3);
}

float UnitreeA1::footRadius() { return A1::constants::FOOT_RADIUS; }

UnitreeA1
UnitreeA1::createDummyA1RobotWithRawState(const Vec3<float> &motorAngles) {
  a1_lcm_msgs::RobotRawState dummyState{
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
  return UnitreeA1(&dummyState);
}

UnitreeA1 UnitreeA1::createDummyA1RobotWithStateEstimates() {
  a1_lcm_msgs::RobotState dummyState{.quaternion = {1, 0, 0, 0},
                                     .gyro = {0, 0, 0},
                                     .accel = {0, 0, 0},
                                     .footForces = {30, 30, 30, 30},
                                     .dq = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                     .tick = 0.0,
                                     .position = {0, 0, 0.25},
                                     .velocityBody = {0, 0, 0}};

  memcpy(dummyState.q, A1::constants::STAND_ANGLES.data(), sizeof(float) * 12);
  return UnitreeA1(&dummyState);
}

} // namespace robots
} // namespace strelka