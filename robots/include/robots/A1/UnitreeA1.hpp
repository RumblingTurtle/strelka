#ifndef UNITREE_A1_H
#define UNITREE_A1_H

#include <robots/A1/kinematics.hpp>

#include <common/Robot.hpp>
#include <common/constants.hpp>
#include <common/macros.hpp>
#include <common/rotation.hpp>
#include <common/typedefs.hpp>
#include <exception>
#include <messages/a1_lcm_msgs/RobotRawState.hpp>
#include <messages/a1_lcm_msgs/RobotState.hpp>

namespace strelka {
namespace robots {
class NoStateEstimateException : std::exception {
  const char *what() {
    return "UnitreeA1 robot has no state estimate. Build a UnitreeA1 class "
           "using a1_lcm_msgs::RobotState";
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
  Mat43<float> _footPositionsTrunkFrame;
  Vec4<bool> _footContacts;
  Mat43<float> _footVelocitiesTrunkFrame;
  Vec3<float> _gyroscopeBodyFrame;
  Vec3<float> _accelerometerBodyFrame;
  Vec3<float> _accelerometerWorldFrame;
  Vec4<float> _footContactHeights;
  Vec12<float> _q;
  Vec12<float> _dq;

  Vec3<float> _positionWorldFrame;
  Vec3<float> _linearVelocityBodyFrame;

  bool hasStateEstimates;

public:
  ~UnitreeA1();
  UnitreeA1(const a1_lcm_msgs::RobotRawState *rawStateMessage);
  UnitreeA1(const a1_lcm_msgs::RobotState *robotStateMessage);

  template <class MessageType>
  void initRawStateEntries(const MessageType *message);
  void initStateEstimateEntries(const a1_lcm_msgs::RobotState *message);

  virtual Vec12<float> q();
  virtual Vec12<float> dq();

  virtual bool footContact(int legId);
  virtual Vec4<bool> footContacts();

  virtual Vec3<float> gyroscopeBodyFrame();
  virtual Vec3<float> accelerometerWorldFrame();

  virtual Vec3<float> positionWorldFrame();
  virtual Vec3<float> linearVelocityBodyFrame();

  virtual Vec3<float> footPositionTrunkFrame(int legId);
  virtual Vec3<float> footVelocityTrunkFrame(int legId);

  virtual Vec3<float> footPositionWorldFrame(int legId);

  virtual Quat<float> bodyToWorldQuat();
  virtual Mat3<float> bodyToWorldMat();

  virtual Vec3<float> rotateBodyToWorldFrame(Vec3<float> vector);
  virtual Vec3<float> rotateWorldToBodyFrame(Vec3<float> vector);

  virtual Vec3<float> transformBodyToWorldFrame(Vec3<float> vector);
  virtual Vec3<float> transformWorldToBodyFrame(Vec3<float> vector);

  virtual Vec3<float> trunkToThighOffset(int legId);
  virtual float footRadius();

  virtual Vec3<float> currentRPY();

  Eigen::Matrix<float, 12, 3> footJacobians();

  static UnitreeA1 createDummyA1RobotWithStateEstimates();
  static UnitreeA1 createDummyA1RobotWithRawState();
};

} // namespace robots
} // namespace strelka
#endif // UNITREE_A1_H