#ifndef WHOLEBODYIMPEDANCE_CTRL_H
#define WHOLEBODYIMPEDANCE_CTRL_H

#include <WBIC/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include <common/Robot.hpp>
#include <common/typedefs.hpp>
#include <messages/WBICCommand.hpp>

namespace strelka {
namespace control {
/**
 * @brief Adapter class for WBIC implementation by MIT
 * https://arxiv.org/abs/1909.06586
 */
class WholeBodyImpulseController {
  LocomotionCtrl<float> *wbic;

public:
  struct WBICParams {
    float Kp[9];
    float Kd[9];
    float Kp_kin[9];
    float floating_W[6];
    float rf_W[12];
    float mu;
    float max_fz;
  };

  struct WBICOutput {
    /**
     * @brief Outputs representing desired angles, joint velocities and torques
     * for provided WBICCommand and robot state
     *
     */
    Vec12<float> q;
    Vec12<float> dq;
    Vec12<float> tau;
  };

  WholeBodyImpulseController(FloatingBaseModel<float> robotModel,
                             WBICParams &parameters);

  ~WholeBodyImpulseController();

  void update(robots::Robot &robot, messages::WBICCommand &command,
              WBICOutput &output);
};

static WholeBodyImpulseController::WBICParams STAND_WBIC_PARAMS = {
    .Kp = {10.0, 10.0, 100.0, 100.0, 100.0, 100.0, 0.0, 0.0, 0.0},
    .Kd = {1.0, 1.0, 10.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0},
    .Kp_kin = {1, 1, 1, 1, 1, 1, 1, 1, 1},
    .floating_W = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
    .rf_W = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    .mu = 0.4,
    .max_fz = 1500,
};

static WholeBodyImpulseController::WBICParams DEFAULT_WBIC_PARAMS = {
    .Kp = {10.0, 10.0, 100.0, 100.0, 100.0, 100.0, 0.0, 0.0, 0.0},
    .Kd = {1.0, 1.0, 10.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0},
    .Kp_kin = {0.0, 0.0, 0.0, 0.8, 0.8, 0.8, 1, 1, 1},
    .floating_W = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
    .rf_W = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
    .mu = 0.4,
    .max_fz = 1500,
};
} // namespace control
} // namespace strelka

#endif // WHOLEBODYIMPEDANCE_CTRL_H