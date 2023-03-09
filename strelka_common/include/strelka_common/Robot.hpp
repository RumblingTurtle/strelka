/**
 * @file Robot.hpp
 * Common robot interface used for state estimation and control
 */
#ifndef ROBOT_H
#define ROBOT_H
#include <strelka_common/typedefs.hpp>

namespace strelka {
namespace robots {
/**
 * @brief Common robot interface used for state estimation and control
 *
 * In order to use state_estimation and control packages you have to provide
 * implementations of this common interface.
 *
 * Leg notations
 * legId - name      - abb.
 *   0   Front right   (FR)
 *   1   Front left    (FL)
 *   2   Rear right    (RR)
 *   3   Rear left     (RL)
 *
 * World frame actually means odometry frame.
 * All coordinate frames are right handed.
 */
class Robot {

public:
  Robot(){};
  virtual ~Robot(){};

  /**
   * Angles and angular velocities for motors
   * 3 per each leg
   * FR-hip FR-thigh FR-knee ....
   */
  virtual Vec12<float> q() = 0;
  virtual Vec12<float> dq() = 0;

  /**
   * @brief Returns contact state of the foot.
   *
   * Contact sensors may not be available on the robot so you might have
   * to implement your own contact estimation implementation
   *
   * @return true if leg is considered to be in contact and false otherwise
   */
  virtual bool footContact(int legId) = 0;
  virtual Vec4<bool> footContacts() = 0;

  /**
   * IMU measurements
   */

  virtual Vec3<float> gyroscopeBodyFrame() = 0;
  virtual Vec3<float> accelerometerWorldFrame() = 0;

  /**
   * Robot odometry from state estimation
   */

  virtual Vec3<float> positionWorldFrame() = 0;
  virtual Vec3<float> linearVelocityBodyFrame() = 0;

  /**
   * These are left to the user implementation to enable chaching possibilites
   */

  virtual Vec3<float> footPositionTrunkFrame(int legId) = 0;
  virtual Vec3<float> footVelocityTrunkFrame(int legId) = 0;
  virtual Vec3<float> footPositionWorldFrame(int legId) = 0;

  virtual Quat<float> bodyToWorldQuat() = 0;
  virtual Mat3<float> bodyToWorldMat() = 0;

  virtual Vec3<float> rotateBodyToWorldFrame(Vec3<float> vector) = 0;
  virtual Vec3<float> rotateWorldToBodyFrame(Vec3<float> vector) = 0;

  virtual Vec3<float> transformBodyToWorldFrame(Vec3<float> vector) = 0;
  virtual Vec3<float> transformWorldToBodyFrame(Vec3<float> vector) = 0;

  /**
   * @brief Offset from the center of the robot body to it's thigh link.
   *
   * Needed for estimation of the next nominal position when planning footholds
   */
  virtual Vec3<float> trunkToThighOffset(int legId) = 0;

  /**
   * @brief Radius of the foot link.
   *
   * Since we are planning movement of the foot center it is important to know
   * it's radius in order to avoid collisions
   */
  virtual float footRadius() = 0;

  virtual Vec3<float> currentRPY() = 0;
};
} // namespace robots
} // namespace strelka
#endif // ROBOT_H