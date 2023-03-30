#ifndef ROBOT_H
#define ROBOT_H
#include <strelka/common/constants.hpp>
#include <strelka/common/macros.hpp>
#include <strelka/common/rotation.hpp>
#include <strelka/common/typedefs.hpp>

#include <strelka_lcm_headers/RobotRawState.hpp>
#include <strelka_lcm_headers/RobotState.hpp>

namespace strelka {
namespace robots {

class NoStateEstimateException : public std::exception {
public:
  const char *what() {
    return "UR has no state estimate. Build a UnitreeA1 class "
           "using strelka_lcm_headers::RobotState";
  }
};
class StatelessRobotException : public std::exception {
public:
  const char *what() {
    return "Rt instance is constructed without any state message "
           ". Build a Robot descendant class "
           "using strelka_lcm_headers::RobotState or "
           "strelka_lcm_headers::RobotRawState";
  }
};
/**
 * @brief Common robot interface used for state estimation and control
 *
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
protected:
  Mat3<float> _bodyToWorldMat;
  Vec3<float> _gyroscopeBodyFrame;
  Vec3<float> _accelerometerBodyFrame;
  Vec3<float> _accelerometerWorldFrame;
  Vec3<float> _positionWorldFrame;
  Vec3<float> _linearVelocityBodyFrame;

  Vec4<float> _bodyToWorldQuat;
  Vec4<bool> _footContacts;
  Vec4<float> _footForces;

  Mat34<float> _footPositionsTrunkFrame;
  Mat34<float> _footVelocitiesTrunkFrame;

  Vec12<float> _q;
  Vec12<float> _dq;
  Eigen::Matrix<float, 12, 3> _footJacobians;

  bool hasStateEstimates_ = false;
  bool hasRawState_ = false;

public:
  explicit Robot(){};
  /**
   * @brief Updates robot state using the state messages
   */
  void update(const strelka_lcm_headers::RobotRawState *rawStateMessage);
  void update(const strelka_lcm_headers::RobotState *robotStateMessage);

  /**
   * @brief RobotState and RobotRawState parsers
   *
   * @tparam MessageType
   * @param message
   */
  template <class MessageType>
  void processRawStateEntries(const MessageType *message);
  void
  processStateEstimateEntries(const strelka_lcm_headers::RobotState *message);
  bool hasStateEstimates();
  bool hasRawState();

  /**
   * @brief Current angle configuration of the motors
   * 3 per each leg
   * FR-hip FR-thigh FR-knee ....
   */
  Vec12<float> q();

  /**
   * @brief Current velocity configuration of the motors
   * 3 per each leg
   * FR-hip FR-thigh FR-knee ....
   */
  Vec12<float> dq();

  /**
   * @brief Returns contact states of the foot.
   *
   * Note that contact sensors may not be available on the robot so you might
   * have to implement your own contact estimation implementation
   *
   * @return Vec4<bool> true if leg is considered to be in contact and false
   * otherwise
   */
  Vec4<bool> footContacts();

  /**
   * @brief Angular velocity taken from IMU
   */
  Vec3<float> gyroscopeBodyFrame();

  /**
   * @brief Acceleration taken from IMU
   */
  Vec3<float> accelerometerWorldFrame();

  /**
   * @brief Robot position in world frame
   */
  Vec3<float> positionWorldFrame();

  /**
   * @brief Robot velocity in body frame
   */
  Vec3<float> linearVelocityBodyFrame();

  /**
   * @brief Foot position in trunk frame
   */
  Vec3<float> footPositionTrunkFrame(int legId);

  /**
   * @brief Foot velocity in trunk frame. Usually calculated as
   * LegJacobian*LegMotorVelocities
   */
  Vec3<float> footVelocityTrunkFrame(int legId);

  /**
   * @brief Foot position in world frame
   */
  Vec3<float> footPositionWorldFrame(int legId);

  /**
   * @brief Body to world rotation quaternion
   * Scalar first
   */
  Vec4<float> bodyToWorldQuat();

  /**
   * @brief Body to world rotation euler matrix
   */
  Mat3<float> bodyToWorldMat();

  /**
   * @brief Body to world rotation euler angles
   */
  Vec3<float> bodyToWorldRPY();

  /**
   * @brief Rotates a given vector by current robot rotation estimate
   *
   * Make sure that the state estimates are available to the robot object!
   *
   * @param vector Vector in robot body frame
   * @return Vec3<float> Vector in world frame
   */
  Vec3<float> rotateBodyToWorldFrame(Vec3<float> vector);

  /**
   * @brief Rotates a given vector by the transpose of the current robot
   * rotation estimate
   *
   * Make sure that the state estimates are available to the robot object!
   *
   * @param vector Vector in robot body frame
   * @return Vec3<float> Vector in world frame
   */
  Vec3<float> rotateWorldToBodyFrame(Vec3<float> vector);

  /**
   * @brief Transforms a given vector from body frame to world frame using state
   * estimates
   *
   * @param vector Vector in robot body frame
   * @return Vec3<float> Vector in world frame
   */
  Vec3<float> transformBodyToWorldFrame(Vec3<float> vector);

  /**
   * @brief Transforms a given vector from world frame to body frame using state
   * estimates
   *
   * @param vector Vector in robot world frame
   * @return Vec3<float> Vector in body frame
   */
  Vec3<float> transformWorldToBodyFrame(Vec3<float> vector);

  /**
   * @brief Joint to end effector velocity mapping matrices
   *
   * @return Eigen::Matrix<float, 12, 3> Stacked 3x3 jacobian matrices for each
   * leg
   */
  Eigen::Matrix<float, 12, 3> footJacobians();

  /*** CONSTANTS ***/

  /**
   * @brief Offset from the center of the robot body to it's thigh link.
   *
   * Needed for estimation of the next nominal position when planning footholds
   */
  virtual Vec3<float> trunkToThighOffset(int legId) = 0;

  /**
   * @brief Offset from the center of the robot body to it's hip link.
   *
   * Needed for estimation of the next nominal position when planning footholds
   */
  virtual Vec3<float> trunkToHipOffset(int legId) = 0;

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

  /*** Required implementations ***/

  /**
   * @brief Checks if the foot position in the world frame is kinematically
   * reachable.
   */
  virtual bool worldFrameIKCheck(Vec3<float> footPositionWorldFrame,
                                 int legId) = 0;

  /**
   * @brief Jacobian matrix calculation implementation
   *
   */
  virtual Mat3<float> footJacobianImpl(int legId) = 0;

  /**
   * @brief Return foot position in trunk frame
   */
  virtual Vec3<float> footPositionTrunkFrameImpl(int legId) = 0;

  /**
   * @brief Estimate if a leg with a given legId is considered to be in contact
   */
  virtual bool estimateContactImpl(int legId) = 0;
};
} // namespace robots
} // namespace strelka
#endif // ROBOT_H