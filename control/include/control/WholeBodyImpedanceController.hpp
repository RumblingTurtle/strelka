#ifndef WHOLEBODYIMPEDANCE_CTRL_H
#define WHOLEBODYIMPEDANCE_CTRL_H

#include <WBIC/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include <common/typedefs.hpp>
#include <control/A1WbicDynamics.hpp>
#include <messages/WBICCommand.hpp>
#include <robots/Robot.hpp>

namespace strelka {
namespace control {
class WholeBodyImpedanceController {
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
    Vec12<float> q;
    Vec12<float> dq;
    Vec12<float> tau;
  };

  WholeBodyImpedanceController(WBICParams &parameters);
  ~WholeBodyImpedanceController();

  void update(robots::Robot &robot, messages::WBICCommand &command,
              WBICOutput &output);
};
} // namespace control
} // namespace strelka

#endif // WHOLEBODYIMPEDANCE_CTRL_H