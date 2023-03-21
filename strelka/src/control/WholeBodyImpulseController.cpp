#include <WBIC/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include <strelka/control/WholeBodyImpulseController.hpp>

typedef LocomotionCtrl<float> FloatLocomotionCtrl;
namespace strelka {

namespace control {

WholeBodyImpulseController::WholeBodyImpulseController(robots::Robot &robot,
                                                       WBICParams &parameters) {
  Quadruped<float> quadruped;

  Vec3<float> bodyCOM = robot.bodyCOMPosition();
  Vec3<float> bodyDimensions = robot.bodyDimensions();
  Vec3<float> legDimensions = robot.legDimensions();

  quadruped._bodyMass = robot.bodyMass();
  quadruped._bodyLength = bodyDimensions(0);
  quadruped._bodyWidth = bodyDimensions(1);
  quadruped._bodyHeight = bodyDimensions(2);

  quadruped._abadLinkLength = legDimensions(0);
  quadruped._hipLinkLength = legDimensions(1);
  quadruped._kneeLinkLength = legDimensions(2);
  quadruped._maxLegLength = legDimensions(1) + legDimensions(2);

  quadruped._abadGearRatio = 9;
  quadruped._kneeLinkY_offset = 0.0;
  quadruped._hipGearRatio = 9;
  quadruped._kneeGearRatio = 9;
  quadruped._motorTauMax = 3.f;
  quadruped._batteryV = 24;
  quadruped._motorKT = .05; // this is flux linkage * pole pairs
  quadruped._motorR = 0.173;
  quadruped._jointDamping = .01;
  quadruped._jointDryFriction = .2;

  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<float> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 0, 0, 0, 0, 0, 0, 0, 0, 0; // ZERO ROTOR INERTIA
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<float> RY = coordinateRotation<float>(CoordinateAxis::Y, M_PI / 2);
  Mat3<float> RX = coordinateRotation<float>(CoordinateAxis::X, M_PI / 2);
  Mat3<float> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<float> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();
  Vec3<float> rotorCOM(0, 0, 0);
  SpatialInertia<float> rotorInertiaX(0.0, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<float> rotorInertiaY(0.0, rotorCOM, rotorRotationalInertiaY);

  // spatial inertias
  Mat3<float> abadRotationalInertia;
  abadRotationalInertia << 402, 0, 0, 0, 691, 0, 0, 0, 487;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<float> abadCOM(-0.003875, 0.001622, 0.000042); // LEFT
  SpatialInertia<float> abadInertia(0.595, abadCOM, abadRotationalInertia);

  Mat3<float> hipRotationalInertia;
  hipRotationalInertia << 5251, 0, 0, 0, 5000, 0, 0, 0, 1110;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<float> hipCOM(-0.003574, -0.019529, -0.030323);
  SpatialInertia<float> hipInertia(0.888, hipCOM, hipRotationalInertia);

  Mat3<float> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 2344, 0, 0, 0, 2360, 0, 0, 0, 31;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<float> kneeCOM(0.007105, -0.000239, -0.096933);
  SpatialInertia<float> kneeInertia(0.151, kneeCOM, kneeRotationalInertia);

  Mat3<float> bodyRotationalInertia = robot.rotationalInertia();

  SpatialInertia<float> bodyInertia(quadruped._bodyMass, bodyCOM,
                                    bodyRotationalInertia);

  quadruped._abadInertia = abadInertia;
  quadruped._hipInertia = hipInertia;
  quadruped._kneeInertia = kneeInertia;
  quadruped._abadRotorInertia = rotorInertiaX;
  quadruped._hipRotorInertia = rotorInertiaY;
  quadruped._kneeRotorInertia = rotorInertiaY;
  quadruped._bodyInertia = bodyInertia;

  // locations
  quadruped._abadRotorLocation = Vec3<float>(0, 0, 0);
  quadruped._abadLocation =
      Vec3<float>(quadruped._bodyLength, quadruped._bodyWidth, 0) * 0.5;
  quadruped._hipLocation = Vec3<float>(0, quadruped._abadLinkLength, 0);
  quadruped._hipRotorLocation = Vec3<float>(0, 0, 0);
  quadruped._kneeLocation = Vec3<float>(0, 0, -quadruped._hipLinkLength);
  quadruped._kneeRotorLocation = Vec3<float>(0, 0, 0);

  wbic = new LocomotionCtrl<float>(
      quadruped.buildModel(), parameters.Kp, parameters.Kd, parameters.Kp_kin,
      parameters.floating_W, parameters.rf_W, parameters.mu, parameters.max_fz);
}

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

WholeBodyImpulseController::~WholeBodyImpulseController() { delete wbic; }
} // namespace control
} // namespace strelka