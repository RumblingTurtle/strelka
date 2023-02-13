#ifndef WHOLEBODYIMPEDANCE_CTRL_H
#define WHOLEBODYIMPEDANCE_CTRL_H

#include <WBIC/Dynamics/UnitreeA1.h>
#include <WBIC/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include <common/typedefs.hpp>

namespace strelka {
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

  struct WBICInput {
    Quat<float> bodyOrientation; // World to body quaternion; r_body =
                                 // R(bodyOrientation)*r_world
    Vec3<float> bodyPosition;    // World coords
    Vec3<float> angularVelocity; // Body frame
    Vec3<float> linearVelocity;  // Body frame
    Vec12<float> q;
    Vec12<float> dq;
    Vec3<float> pBody_RPY_des; // World to body quaternion; r_body =
                               // R(bodyOrientation)*r_world
    Vec3<float> vBody_Ori_des; // Body frame
    Vec3<float> pBody_des;     // World frame
    Vec3<float> vBody_des;     // World frame
    Vec3<float> aBody_des;     // World frame
    Vec12<float> pFoot_des;    // World frame
    Vec12<float> vFoot_des;    // World frame
    Vec12<float> aFoot_des;    // World frame
    Vec4<float> contact_state;
    Vec12<float> Fr_des_MPC; // World frame
  };

  struct WBICOutput {
    Vec12<float> q;
    Vec12<float> dq;
    Vec12<float> tau;
  };

  WholeBodyImpedanceController(WBICParams &parameters);
  ~WholeBodyImpedanceController();

  void update(WBICInput &input, WBICOutput &output);
};
} // namespace strelka

#endif // WHOLEBODYIMPEDANCE_CTRL_H