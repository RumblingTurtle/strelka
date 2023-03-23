#ifndef UNITREE_A1_H
#define UNITREE_A1_H

#include <strelka/robots/A1/constants.hpp>
#include <strelka/robots/A1/kinematics.hpp>

#include <exception>
#include <strelka/common/constants.hpp>
#include <strelka/common/macros.hpp>
#include <strelka/common/rotation.hpp>
#include <strelka/common/typedefs.hpp>
#include <strelka/robots/Robot.hpp>

namespace strelka {
namespace robots {
class NoStateEstimateException : public std::exception {
public:
  const char *what() {
    return "UnitreeA1 robot has no state estimate. Build a UnitreeA1 class "
           "using strelka_lcm_headers::RobotState";
  }
};
class StatelessRobotException : public std::exception {
public:
  const char *what() {
    return "UnitreeA1 robot instance is constructed without any state message "
           ". Build a UnitreeA1 class "
           "using strelka_lcm_headers::RobotState or "
           "strelka_lcm_headers::RobotRawState";
  }
};
/**
 * @brief Example of Robot interface implementation using Unitree A1 robot
 *
 */
class UnitreeA1 : public Robot {

  Eigen::Matrix<float, 12, 3> _footJacobians;
  Mat3<float> _bodyToWorldMat;
  Quat<float> _bodyToWorldQuat;
  Vec4<bool> _footContacts;
  Mat34<float> _footPositionsTrunkFrame;
  Mat34<float> _footVelocitiesTrunkFrame;
  Vec3<float> _gyroscopeBodyFrame;
  Vec3<float> _accelerometerBodyFrame;
  Vec3<float> _accelerometerWorldFrame;
  Vec4<float> _footContactHeights;
  Vec12<float> _q;
  Vec12<float> _dq;

  Vec3<float> _positionWorldFrame;
  Vec3<float> _linearVelocityBodyFrame;

  bool hasStateEstimates = false;
  bool hasRawState = false;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Default constructed instance is used for constants
  explicit UnitreeA1();
  UnitreeA1(const strelka_lcm_headers::RobotRawState *rawStateMessage);
  UnitreeA1(const strelka_lcm_headers::RobotState *robotStateMessage);

  Vec12<float> q() override;
  Vec12<float> dq() override;

  Vec4<bool> footContacts() override;

  Vec3<float> gyroscopeBodyFrame() override;
  Vec3<float> accelerometerWorldFrame() override;

  Vec3<float> positionWorldFrame() override;
  Vec3<float> linearVelocityBodyFrame() override;

  Vec3<float> footPositionTrunkFrame(int legId) override;
  Vec3<float> footVelocityTrunkFrame(int legId) override;

  Vec3<float> footPositionWorldFrame(int legId) override;

  Quat<float> bodyToWorldQuat() override;
  Mat3<float> bodyToWorldMat() override;

  Vec3<float> rotateBodyToWorldFrame(Vec3<float> vector) override;
  Vec3<float> rotateWorldToBodyFrame(Vec3<float> vector) override;

  Vec3<float> transformBodyToWorldFrame(Vec3<float> vector) override;
  Vec3<float> transformWorldToBodyFrame(Vec3<float> vector) override;
  Eigen::Matrix<float, 12, 3> footJacobians() override;

  Vec3<float> bodyToWorldRPY() override;

  bool worldFrameIKCheck(Vec3<float> footPositionWorldFrame,
                         int legId) override;

  Vec3<float> trunkToThighOffset(int legId) override;

  const float footRadius() override;
  const float trunkMass() override;
  const float robotMass() override;
  Vec3<float> bodyToComOffset() override;
  Vec3<float> bodyDimensions() override;
  Vec3<float> legDimensions() override;
  Vec3<float> positionGains() override;
  Vec3<float> dampingGains() override;
  Vec3<float> initAngles() override;
  Vec3<float> standAngles() override;
  Mat3<float> rotationalInertia() override;

  template <class MessageType>
  void initRawStateEntries(const MessageType *message);
  void initStateEstimateEntries(const strelka_lcm_headers::RobotState *message);

  static UnitreeA1 &createDummyA1RobotWithStateEstimates();
  static UnitreeA1 &createDummyA1RobotWithRawState(
      Vec3<float> motorAngles = A1::constants::STAND_ANGLES);
};

} // namespace robots
} // namespace strelka
#endif // UNITREE_A1_H