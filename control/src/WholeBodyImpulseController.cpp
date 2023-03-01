#include <control/WholeBodyImpulseController.hpp>

namespace strelka {

namespace control {

WholeBodyImpulseController::WholeBodyImpulseController(
    FloatingBaseModel<float> robotModel, WBICParams &parameters) {
  wbic = new LocomotionCtrl<float>(
      robotModel, parameters.Kp, parameters.Kd, parameters.Kp_kin,
      parameters.floating_W, parameters.rf_W, parameters.mu, parameters.max_fz);
}

WholeBodyImpulseController::~WholeBodyImpulseController() { delete wbic; }

void WholeBodyImpulseController::update(robots::Robot &robot,
                                        messages::WBICCommand &command,
                                        WBICOutput &output) {

  wbic->update(robot.bodyToWorldQuat(), robot.positionWorldFrame(),
               robot.gyroscopeBodyFrame(), robot.linearVelocityBodyFrame(),
               robot.q(), robot.dq(), command.desiredBodyRPY(),
               command.desiredBodyAngularVelocity(),
               command.desiredBodyPosition(), command.desiredBodyVelocity(),
               command.desiredBodyAcceleration(), command.desiredFootPosition(),
               command.desiredFootVelocity(), command.desiredFootAcceleration(),
               command.desiredContactState(), command.desiredFootForceWorld());

  output.q = wbic->get_qDes();
  output.dq = wbic->get_dqDes();
  wbic->compute_tau();
  output.tau = wbic->getTau();
}
} // namespace control
} // namespace strelka