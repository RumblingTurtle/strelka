#include <control/WholeBodyImpedanceController.hpp>

namespace strelka {

WholeBodyImpedanceController::WholeBodyImpedanceController(
    WBICParams &parameters) {
  wbic = new LocomotionCtrl<float>(buildA1<float>().buildModel(), parameters.Kp,
                                   parameters.Kd, parameters.Kp_kin,
                                   parameters.floating_W, parameters.rf_W,
                                   parameters.mu, parameters.max_fz);
}

WholeBodyImpedanceController::~WholeBodyImpedanceController() { delete wbic; }

void WholeBodyImpedanceController::update(WBICInput &input,
                                          WBICOutput &output) {
  wbic->update(input.bodyOrientation, input.bodyPosition, input.angularVelocity,
               input.linearVelocity, input.q, input.dq, input.pBody_RPY_des,
               input.vBody_Ori_des, input.pBody_des, input.vBody_des,
               input.aBody_des, input.pFoot_des, input.vFoot_des,
               input.aFoot_des, input.contact_state, input.Fr_des_MPC);

  output.tau = wbic->get_tau();
  output.q = wbic->get_qDes();
  output.dq = wbic->get_dqDes();
}
} // namespace strelka