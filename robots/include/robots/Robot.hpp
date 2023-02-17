#ifndef ROBOT_H
#define ROBOT_H
#include <common/typedefs.hpp>

namespace strelka {
namespace robots {
class Robot {
public:
  Robot(){};
  virtual ~Robot(){};

  virtual Vec12<float> q() = 0;
  virtual Vec12<float> dq() = 0;

  virtual float footContact(int legId) = 0;

  virtual Vec3<float> gyroscopeBodyFrame() = 0;
  virtual Vec3<float> accelerometerWorldFrame() = 0;

  virtual Vec3<float> positionWorldFrame() = 0;
  virtual Vec3<float> linearVelocityBodyFrame() = 0;

  virtual Vec3<float> footPositionTrunkFrame(int legId) = 0;
  virtual Vec3<float> footVelocityTrunkFrame(int legId) = 0;

  virtual Vec3<float> footPositionWorldFrame(int legId) = 0;

  virtual float footContactHeightWorldFrame(int legId) = 0;
  virtual Eigen::Matrix<float, 12, 3> footJacobians() = 0;

  virtual Quat<float> bodyToWorldQuat() = 0;
  virtual Mat3<float> bodyToWorldMat() = 0;

  virtual Vec3<float> rotateBodyToWorldFrame(Vec3<float> vector) = 0;
  virtual Vec3<float> rotateWorldToBodyFrame(Vec3<float> vector) = 0;

  virtual Vec3<float> transformBodyToWorldFrame(Vec3<float> vector) = 0;
  virtual Vec3<float> transformWorldToBodyFrame(Vec3<float> vector) = 0;

  virtual Vec3<float> currentRPY() = 0;
};
} // namespace robots
} // namespace strelka
#endif // ROBOT_H