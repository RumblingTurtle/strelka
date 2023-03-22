#ifndef MOVE_TO_INTERFACE_H
#define MOVE_TO_INTERFACE_H

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <math.h>
#include <strelka/common/Robot.hpp>
#include <strelka/common/constants.hpp>
#include <strelka/common/typedefs.hpp>
#include <strelka_lcm_headers/RobotLowCommand.hpp>
#include <strelka_lcm_headers/RobotRawState.hpp>

#include <strelka/robots/Robots.hpp>

#define SIMULATION_TIME_STEP 0.001
#define SIMULATION_BASE_RATE 1000

namespace strelka {
namespace interfaces {

class RobotStateTopicDoesNotExist : public std::exception {
public:
  const char *what() {
    return "Robot state topic set for in MoveToInterface does not exist";
  }
};

/**
 * @brief Generic interface for any 12 motor robot used to interpolate between
 * current and desired angles,joint velocities and torques
 *
 */
template <class RobotClass> class MoveToInterface {
  // Instance needed only for gain constants queriying
  robots::Robot &robotInstance;
  strelka_lcm_headers::RobotLowCommand commandMessage;
  lcm::LCM lcm;

  class MoveToHandle {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    float moveTime;
    float timeLeft;
    float prevTick;
    const char *stateTopicName;
    lcm::LCM lcm;

    Vec12<float> startAngles;
    Vec12<float> desiredAngles;
    MoveToInterface<RobotClass> &interface;

  public:
    MoveToHandle(MoveToInterface &interface, const char *stateTopicName);

    void run(float moveTime, const Vec12<float> &desiredAngles);
    void moveHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                    const strelka_lcm_headers::RobotRawState *data);

    void initStartAnglesHandle(const lcm::ReceiveBuffer *rbuf,
                               const std::string &chan,
                               const strelka_lcm_headers::RobotRawState *data);

    Vec12<float> getStartAngles();
  };

public:
  MoveToInterface(robots::Robot &robot);

  void sendCommandMessage(const Vec12<float> &command);
  void setTorques(const Vec12<float> &torques);

  void setAngles(const Vec12<float> &q);
  void setAngles(const Vec12<float> &q, const Vec12<float> &dq);

  /**
   * @brief Get current angle configuration
   *
   * @return Vec12<float> current motor angles
   */
  Vec12<float> getAngles();

  /**
   * @brief Moves the robot to specified angle configuration over moveTime
   * seconds
   *
   * @param angles 12 desired motor angles
   * @param moveTime Time to move the robot from current to specified
   * configuration
   */
  void moveTo(const Vec12<float> &angles, float moveTime);

  /**
   * @brief Moves each of the robot legs to specified angle configuration over
   * moveTime seconds
   *
   * @param angles 3 desired motor angles per each leg
   * @param moveTime Time to move the robot from current to specified
   * configuration
   */
  void moveTo(const Vec3<float> &angles, float moveTime);

  /**
   * @brief Moves the robot to initial configuration over moveTime seconds
   * Initial configuration might be lying on the ground or calibration
   * configuration
   *
   * @param moveTime Time to move the robot from current to initial
   * configuration
   */
  void moveToInit(float moveTime = 3.0);

  /**
   * @brief Moves the robot to stand configuration over moveTime seconds
   *
   * @param moveTime Time to move the robot from current to stand
   * configuration
   */
  void moveToStand(float moveTime = 3.0);
};
} // namespace interfaces
} // namespace strelka

#endif // MOVE_TO_INTERFACE_H
