#ifndef GAZEBO_INTERFACE_H
#define GAZEBO_INTERFACE_H

#include <common/A1/constants.hpp>
#include <common/typedefs.hpp>
#include <interfaces/QuadrupedInterface.hpp>
#include <lcm/lcm-cpp.hpp>
#include <math.h>
#include <messages/a1_lcm_msgs/RobotLowCommand.hpp>
#include <messages/a1_lcm_msgs/RobotRawState.hpp>

#define GAZEBO_BASE_TIME_STEP 0.001
#define GAZEBO_BASE_RATE 1000

namespace strelka {
class GazeboInterface : public QuadrupedInterface {
  lcm::LCM lcm;
  a1_lcm_msgs::RobotLowCommand commandMessage;

  struct MoveToHandle {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    float moveTime;
    float timeLeft;
    float prevTick;
    bool firstHandle;

    Eigen::VectorXf startAngles;
    const Eigen::VectorXf &desiredAngles;
    GazeboInterface &interface;

    MoveToHandle(float moveTime, const Eigen::VectorXf &desiredAngles,
                 GazeboInterface &interface);

    void handle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                const a1_lcm_msgs::RobotRawState *data);
  };

public:
  explicit GazeboInterface() {}

  virtual void sendCommandMessage(const Eigen::VectorXf &command) override;

  virtual void setTorques(const Eigen::VectorXf &torques) override;

  virtual void setAngles(const Eigen::VectorXf &q) override;

  virtual void setAngles(const Eigen::VectorXf &q,
                         const Eigen::VectorXf &dq) override;

  virtual void moveTo(const Eigen::VectorXf &angles, float moveTime) override;

  virtual void moveToInit(float moveTime = 3.0) override;

  virtual void moveToStand(float moveTime = 3.0) override;
};

} // namespace strelka

#endif // GAZEBO_INTERFACE_H
