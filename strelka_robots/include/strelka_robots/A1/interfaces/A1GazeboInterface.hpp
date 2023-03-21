#ifndef A1_GAZEBO_INTERFACE_H
#define A1_GAZEBO_INTERFACE_H

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <math.h>
#include <strelka/common/QuadrupedInterface.hpp>
#include <strelka/common/constants.hpp>
#include <strelka/common/typedefs.hpp>
#include <strelka_lcm_headers/RobotLowCommand.hpp>
#include <strelka_lcm_headers/RobotRawState.hpp>
#include <strelka_robots/A1/constants.hpp>

#define GAZEBO_BASE_TIME_STEP 0.001
#define GAZEBO_BASE_RATE 1000

namespace strelka {
namespace interfaces {

class RobotStateTopicDoesNotExist : public std::exception {
public:
  const char *what() {
    return "Robot state topic set for in A1GazeboInterface does not exist";
  }
};

class A1GazeboInterface : public QuadrupedInterface {
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
    A1GazeboInterface &interface;

  public:
    MoveToHandle(A1GazeboInterface &interface, const char *stateTopicName);

    void run(float moveTime, const Vec12<float> &desiredAngles);
    void moveHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                    const strelka_lcm_headers::RobotRawState *data);

    void initStartAnglesHandle(const lcm::ReceiveBuffer *rbuf,
                               const std::string &chan,
                               const strelka_lcm_headers::RobotRawState *data);

    Vec12<float> getInitAngles();
  };

public:
  explicit A1GazeboInterface() {}

  void sendCommandMessage(const Vec12<float> &command) override;

  void setTorques(const Vec12<float> &torques) override;

  void setAngles(const Vec12<float> &q) override;

  void setAngles(const Vec12<float> &q, const Vec12<float> &dq) override;

  Vec12<float> getAngles() override;

  void moveTo(const Vec12<float> &angles, float moveTime) override;

  void moveTo(const Vec3<float> &angles, float moveTime);

  void moveToInit(float moveTime = 3.0) override;

  void moveToStand(float moveTime = 3.0) override;
};
} // namespace interfaces
} // namespace strelka

#endif // GAZEBO_INTERFACE_H
