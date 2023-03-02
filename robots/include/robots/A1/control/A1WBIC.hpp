#ifndef A1WBIC_H
#define A1WBIC_H

#include <common/typedefs.hpp>
#include <common/utilities.hpp>
#include <control/WholeBodyImpulseController.hpp>
#include <messages/a1_lcm_msgs/RobotLowCommand.hpp>
#include <messages/a1_lcm_msgs/RobotState.hpp>
#include <messages/a1_lcm_msgs/WbicCommand.hpp>
#include <robots/A1/UnitreeA1.hpp>
#include <robots/A1/control/A1WbicDynamics.hpp>
#include <robots/A1/kinematics.hpp>
#include <state_estimation/SlowdownEstimator.hpp>

namespace strelka {
namespace control {
class A1WBIC {
  lcm::LCM lcm;
  control::WholeBodyImpulseController controller;

  a1_lcm_msgs::RobotLowCommand *lowCommandMessage;
  a1_lcm_msgs::WbicCommand *currentCommandMessage;

  bool firstCommandRecieved;

  ChronoTimePoint lastCommandTimestamp;

  lcm::Subscription *stateSub;
  lcm::Subscription *commandSub;

  void stateHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                   const a1_lcm_msgs::RobotState *messageIn);

  void commandHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                     const a1_lcm_msgs::WbicCommand *commandMsg);

  void initialize();

public:
  A1WBIC(control::WholeBodyImpulseController::WBICParams &parameters);
  ~A1WBIC();
  void processLoop();
};
} // namespace control
} // namespace strelka
#endif