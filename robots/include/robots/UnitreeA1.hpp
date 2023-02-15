#ifndef UNITREE_A1_H
#define UNITREE_A1_H

#include <common/A1/constants.hpp>
#include <common/A1/kinematics.hpp>
#include <common/constants.hpp>
#include <common/macros.hpp>
#include <common/rotation.hpp>
#include <common/typedefs.hpp>
#include <messages/a1_lcm_msgs/RobotRawState.hpp>
#include <messages/a1_lcm_msgs/RobotState.hpp>
#include <robots/Robot.hpp>

namespace strelka {
namespace robots {
class UnitreeA1 : public Robot {
  Eigen::Matrix<float, 12, 3> _footJacobians;
  Mat3<float> _bodyToWorldMat;
  Vec43<float> _footPositionsTrunkFrame;
  Vec4<float> _footContacts;
  Vec43<float> _footVelocitiesTrunkFrame;
  Vec3<float> _gyroscopeBodyFrame;
  Vec3<float> _accelerometerBodyFrame;
  Vec3<float> _accelerometerWorldFrame;
  Vec4<float> _footContactHeights;
  Vec12<float> _q;
  Vec12<float> _dq;

public:
  ~UnitreeA1();
  UnitreeA1(a1_lcm_msgs::RobotRawState const *rawStateMessage);

  Mat3<float> bodyToWorldMat();
  Vec3<float> rotateBodyToWorldFrame(Vec3<float> vector);
  Vec3<float> rotateWorldToBodyFrame(Vec3<float> vector);
  float footContact(int legId);
  Vec3<float> footPositionTrunkFrame(int legId);
  Vec3<float> footVelocityTrunkFrame(int legId);
  Vec3<float> gyroscopeBodyFrame();
  Vec3<float> accelerometerWorldFrame();
  Eigen::Matrix<float, 12, 3> footJacobians();
  float footContactHeightWorldFrame(int legId);
};

UnitreeA1 createDummyA1RobotWithRawState();
} // namespace robots
} // namespace strelka
#endif // UNITREE_A1_H