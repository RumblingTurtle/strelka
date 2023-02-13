#ifndef GAZEBO_INTERFACE_H
#define GAZEBO_INTERFACE_H

#include <a1_lcm_msgs/RobotLowCommand.hpp>
#include <a1_lcm_msgs/RobotRawState.hpp>
#include <common/A1/constants.hpp>
#include <common/typedefs.hpp>
#include <interfaces/QuadrupedInterface.hpp>
#include <lcm/lcm-cpp.hpp>
#include <math.h>

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

  void sendCommandMessage(const Eigen::VectorXf &command) override;

  void setTorques(const Eigen::VectorXf &torques) override;

  void setAngles(const Eigen::VectorXf &q) override;

  void setAngles(const Eigen::VectorXf &q, const Eigen::VectorXf &dq) override;

  void moveTo(const Eigen::VectorXf &angles, float moveTime) override;

  void moveToInit(float moveTime = 3.0) override;

  void moveToStand(float moveTime = 3.0) override;

  /*
    TODO: void slow_down_sim(self,sim_slowdown=1,sim_slowdown_type = "rate");
  */
};

} // namespace strelka

#endif // GAZEBO_INTERFACE_H
