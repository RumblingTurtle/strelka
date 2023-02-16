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

void WholeBodyImpedanceController::update(robots::Robot &robot,
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

  output.tau = wbic->getTau();
  output.q = wbic->get_qDes();
  output.dq = wbic->get_dqDes();
}
} // namespace strelka