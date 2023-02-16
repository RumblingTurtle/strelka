#ifndef A1WBIC_H
#define A1WBIC_H

#include <common/A1/constants.hpp>
#include <common/A1/kinematics.hpp>
#include <common/typedefs.hpp>
#include <control/WholeBodyImpedanceController.hpp>
#include <messages/a1_lcm_msgs/RobotLowCommand.hpp>
#include <messages/a1_lcm_msgs/RobotState.hpp>
#include <messages/a1_lcm_msgs/WbicCommand.hpp>
#include <robots/UnitreeA1.hpp>
#include <state_estimation/SlowdownEstimator.hpp>

namespace strelka {
static control::WholeBodyImpedanceController::WBICParams A1_STAND_WBIC_PARAMS =
    {
        .Kp = {10.0, 10.0, 100.0, 100.0, 100.0, 100.0, 0.0, 0.0, 0.0},
        .Kd = {1.0, 1.0, 10.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0},
        .Kp_kin = {1, 1, 1, 1, 1, 1, 1, 1, 1},
        .floating_W = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
        .rf_W = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        .mu = 0.4,
        .max_fz = 1500,
};

static control::WholeBodyImpedanceController::WBICParams
    A1_DEFAULT_WBIC_PARAMS = {
        .Kp = {10.0, 10.0, 100.0, 100.0, 100.0, 100.0, 0.0, 0.0, 0.0},
        .Kd = {1.0, 1.0, 10.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0},
        .Kp_kin = {0, 0, 0, 0.8, 0.8, 0.8, 1, 1, 1},
        .floating_W = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
        .rf_W = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
        .mu = 0.4,
        .max_fz = 1500,
};

class A1WBIC {
  lcm::LCM lcm;
  control::WholeBodyImpedanceController controller;

  a1_lcm_msgs::RobotLowCommand *commandMessage;
  a1_lcm_msgs::RobotState *currentState;

  lcm::Subscription *stateSub;
  lcm::Subscription *commandSub;

  void stateHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                   const a1_lcm_msgs::RobotState *messageIn);

  void commandHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                     const a1_lcm_msgs::WbicCommand *commandMsg);

  void initialize();

public:
  A1WBIC(control::WholeBodyImpedanceController::WBICParams &parameters);
  ~A1WBIC();
  void processLoop();
};

} // namespace strelka
#endif