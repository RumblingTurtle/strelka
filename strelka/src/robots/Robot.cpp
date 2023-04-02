#include <strelka/robots/Robot.hpp>

namespace strelka {

namespace robots {

void Robot::update(const strelka_lcm_headers::RobotState *robotStateMessage) {
  processRawStateEntries(robotStateMessage);
  processStateEstimateEntries(robotStateMessage);
}

void Robot::update(const strelka_lcm_headers::RobotRawState *rawStateMessage) {
  processRawStateEntries(rawStateMessage);
}

template <class MessageType>
void Robot::processRawStateEntries(const MessageType *message) {
  hasRawState_ = true;
  hasStateEstimates_ = false;
  _bodyToWorldQuat =
      Eigen::Map<const Eigen::Matrix<float, 4, 1>>(message->quaternion, 4);

  rotation::quat2rot(_bodyToWorldQuat, _bodyToWorldMat);

  _q = Eigen::Map<const Vec12<float>>(message->q, 12);
  _dq = Eigen::Map<const Vec12<float>>(message->dq, 12);

  _gyroscopeBodyFrame = Eigen::Map<const Vec3<float>>(message->gyro, 3);
  _accelerometerBodyFrame = Eigen::Map<const Vec3<float>>(message->accel, 3);

  _footForces = Eigen::Map<const Vec4<float>>(message->footForces, 4);

  FOR_EACH_LEG {
    _footPositionsTrunkFrame.col(LEG_ID) = footPositionTrunkFrameImpl(LEG_ID);

    _footJacobians.block<3, 3>(LEG_ID * 3, 0) = footJacobianImpl(LEG_ID);

    _footVelocitiesTrunkFrame.col(LEG_ID) =
        (_footJacobians.block<3, 3>(LEG_ID * 3, 0) *
         _dq.block<3, 1>(LEG_ID * 3, 0))
            .transpose();

    _footContacts(LEG_ID) = estimateContactImpl(LEG_ID);
  }

  _accelerometerWorldFrame =
      rotateBodyToWorldFrame(_accelerometerBodyFrame) +
      Vec3<float>{0.0f, 0.0f, constants::GRAVITY_CONSTANT};
}

void Robot::processStateEstimateEntries(
    const strelka_lcm_headers::RobotState *message) {
  hasStateEstimates_ = true;
  _positionWorldFrame = Eigen::Map<const Vec3<float>>(message->position, 3);
  _linearVelocityBodyFrame =
      Eigen::Map<const Vec3<float>>(message->velocityBody, 3);
}

bool Robot::hasStateEstimates() { return hasStateEstimates_; }

bool Robot::hasRawState() { return hasRawState_; }

Eigen::Matrix<float, 12, 3> Robot::footJacobians() {
  if (!hasRawState_) {
    throw StatelessRobotException();
  }
  return _footJacobians;
}

Mat3<float> Robot::bodyToWorldMat() {
  if (!hasRawState_) {
    throw StatelessRobotException();
  }
  return _bodyToWorldMat;
}

Vec3<float> Robot::rotateBodyToWorldFrame(Vec3<float> vector) {
  if (!hasRawState_) {
    throw StatelessRobotException();
  }
  return _bodyToWorldMat * vector;
}

Vec3<float> Robot::rotateWorldToBodyFrame(Vec3<float> vector) {
  if (!hasRawState_) {
    throw StatelessRobotException();
  }
  return _bodyToWorldMat.transpose() * vector;
}

Vec4<bool> Robot::footContacts() {
  if (!hasRawState_) {
    throw StatelessRobotException();
  }
  return _footContacts;
}

Vec3<float> Robot::footPositionTrunkFrame(int legId) {
  if (!hasRawState_) {
    throw StatelessRobotException();
  }
  return _footPositionsTrunkFrame.col(legId);
}

Vec3<float> Robot::footVelocityTrunkFrame(int legId) {
  if (!hasRawState_) {
    throw StatelessRobotException();
  }
  return _footVelocitiesTrunkFrame.col(legId);
}
Vec3<float> Robot::gyroscopeBodyFrame() {
  if (!hasRawState_) {
    throw StatelessRobotException();
  }
  return _gyroscopeBodyFrame;
}

Vec3<float> Robot::accelerometerWorldFrame() {
  if (!hasRawState_) {
    throw StatelessRobotException();
  }
  return _accelerometerWorldFrame;
}

Vec12<float> Robot::q() {
  if (!hasRawState_) {
    throw StatelessRobotException();
  }
  return _q;
};
Vec12<float> Robot::dq() {
  if (!hasRawState_) {
    throw StatelessRobotException();
  }
  return _dq;
};

Vec4<float> Robot::bodyToWorldQuat() {
  if (!hasRawState_) {
    throw StatelessRobotException();
  }
  return _bodyToWorldQuat;
};

Vec3<float> Robot::bodyToWorldRPY() {
  return rotation::quat2euler(bodyToWorldQuat());
}

Vec3<float> Robot::footPositionWorldFrame(int legId) {
  return rotateBodyToWorldFrame(footPositionTrunkFrame(legId)) +
         positionWorldFrame();
}

Vec3<float> Robot::transformBodyToWorldFrame(Vec3<float> vector) {
  return rotateBodyToWorldFrame(vector) + positionWorldFrame();
}

Vec3<float> Robot::transformWorldToBodyFrame(Vec3<float> vector) {
  return rotateWorldToBodyFrame(vector - positionWorldFrame());
}

Vec3<float> Robot::positionWorldFrame() {
  if (!hasStateEstimates_) {
    throw NoStateEstimateException();
  }
  return _positionWorldFrame;
};

Vec3<float> Robot::linearVelocityBodyFrame() {
  if (!hasStateEstimates_) {
    throw NoStateEstimateException();
  }
  return _linearVelocityBodyFrame;
};

} // namespace robots
} // namespace strelka