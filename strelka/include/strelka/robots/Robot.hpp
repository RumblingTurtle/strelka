/**
 * @file Robot.hpp
 * Common robot interface used for state estimation and control
 */
#ifndef ROBOT_H
#define ROBOT_H
#include <strelka/common/typedefs.hpp>
#include <strelka_lcm_headers/RobotRawState.hpp>
#include <strelka_lcm_headers/RobotState.hpp>

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
   * @brief Current angle configuration of the motors
   * 3 per each leg
   * FR-hip FR-thigh FR-knee ....
   */
  virtual Vec12<float> q() = 0;

  /**
   * @brief Current velocity configuration of the motors
   * 3 per each leg
   * FR-hip FR-thigh FR-knee ....
   */
  virtual Vec12<float> dq() = 0;

  /**
   * @brief Returns contact states of the foot.
   *
   * Note that contact sensors may not be available on the robot so you might
   * have to implement your own contact estimation implementation
   *
   * @return Vec4<bool> true if leg is considered to be in contact and false
   * otherwise
   */
  virtual Vec4<bool> footContacts() = 0;

  /**
   * @brief Angular velocity taken from IMU
   */
  virtual Vec3<float> gyroscopeBodyFrame() = 0;

  /**
   * @brief Acceleration taken from IMU
   */
  virtual Vec3<float> accelerometerWorldFrame() = 0;

  /**
   * @brief Robot position in world frame
   */
  virtual Vec3<float> positionWorldFrame() = 0;

  /**
   * @brief Robot velocity in body frame
   */
  virtual Vec3<float> linearVelocityBodyFrame() = 0;

  /**
   * @brief Foot position in trunk frame
   */
  virtual Vec3<float> footPositionTrunkFrame(int legId) = 0;

  /**
   * @brief Foot velocity in trunk frame. Usually calculated as
   * LegJacobian*LegMotorVelocities
   */
  virtual Vec3<float> footVelocityTrunkFrame(int legId) = 0;

  /**
   * @brief Foot position in world frame
   */
  virtual Vec3<float> footPositionWorldFrame(int legId) = 0;

  /**
   * @brief Body to world rotation quaternion
   */
  virtual Quat<float> bodyToWorldQuat() = 0;

  /**
   * @brief Body to world rotation euler matrix
   */
  virtual Mat3<float> bodyToWorldMat() = 0;

  /**
   * @brief Body to world rotation euler angles
   */
  virtual Vec3<float> bodyToWorldRPY() = 0;

  /**
   * @brief Rotates a given vector by current robot rotation estimate
   *
   * Make sure that the state estimates are available to the robot object!
   *
   * @param vector Vector in robot body frame
   * @return Vec3<float> Vector in world frame
   */
  virtual Vec3<float> rotateBodyToWorldFrame(Vec3<float> vector) = 0;

  /**
   * @brief Rotates a given vector by the transpose of the current robot
   * rotation estimate
   *
   * Make sure that the state estimates are available to the robot object!
   *
   * @param vector Vector in robot body frame
   * @return Vec3<float> Vector in world frame
   */
  virtual Vec3<float> rotateWorldToBodyFrame(Vec3<float> vector) = 0;

  /**
   * @brief Transforms a given vector from body frame to world frame using state
   * estimates
   *
   * @param vector Vector in robot body frame
   * @return Vec3<float> Vector in world frame
   */
  virtual Vec3<float> transformBodyToWorldFrame(Vec3<float> vector) = 0;

  /**
   * @brief Transforms a given vector from world frame to body frame using state
   * estimates
   *
   * @param vector Vector in robot world frame
   * @return Vec3<float> Vector in body frame
   */
  virtual Vec3<float> transformWorldToBodyFrame(Vec3<float> vector) = 0;

  /**
   * @brief Checks if the foot position in the world frame is kinematically
   * reachable.
   */
  virtual bool worldFrameIKCheck(Vec3<float> footPositionWorldFrame,
                                 int legId) = 0;

  /**
   * @brief Joint to end effector velocity mapping matrices
   *
   * @return Eigen::Matrix<float, 12, 3> Stacked 3x3 jacobian matrices for each
   * leg
   */
  virtual Eigen::Matrix<float, 12, 3> footJacobians() = 0;

  /*** CONSTANTS ***/

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
  virtual const float footRadius() = 0;

  /**
   * @brief Center of mass offset of the robot
   */
  virtual Vec3<float> bodyToComOffset() = 0;

  /**
   * @brief X Y Z dimensions of the body link
   */
  virtual Vec3<float> bodyDimensions() = 0;

  /**
   * @brief Link lengths in the order: Thigh hip knee
   */
  virtual Vec3<float> legDimensions() = 0;

  /**
   * @brief Trunk link mass
   */
  virtual const float trunkMass() = 0;

  /**
   * @brief Mass of the whole robot
   */
  virtual const float robotMass() = 0;

  /**
   * @brief Rotational inertia matrix
   */
  virtual Mat3<float> rotationalInertia() = 0;

  /**
   * @brief Angle configuration of the foot for initialization state. Usually
   * represented by the angles during encoder calibration stage.
   */
  virtual Vec3<float> initAngles() = 0;

  /**
   * @brief Angle configuration for standing
   */
  virtual Vec3<float> standAngles() = 0;

  /**
   * @brief Position gains for servos
   * Note that the same values are used for every triplet representing each leg
   */
  virtual Vec3<float> positionGains() = 0;

  /**
   * @brief Damping gains for servos
   * Note that the same values are used for every triplet representing each leg
   */
  virtual Vec3<float> dampingGains() = 0;
};
} // namespace robots
} // namespace strelka
#endif // ROBOT_H