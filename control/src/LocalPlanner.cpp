#include <control/LocalPlanner.hpp>

namespace strelka {
namespace control {

LocalPlanner::LocalPlanner(double mpcBodyMass, const Vec3<double> bodyInertia,
                           float stepDt, int horizonSteps)
    : scheduler(GAITS::TROT), bodyPlanner(), footPlanner(scheduler),
      _stepDt(stepDt), _horizonSteps(horizonSteps), _mpcBodyMass(mpcBodyMass),
      mpc(mpcBodyMass, bodyInertia, horizonSteps, stepDt) {
  FOR_EACH_LEG { _footState[LEG_ID] = 1; }
  _mpcForces.setZero();
  _desiredFootP.setZero();
  _desiredFootV.setZero();
  _desiredFootA.setZero();

  _desiredRpy.setZero();
  _desiredPositionBody.setZero();
  _desiredAngularVelocity.setZero();
  _desiredVelocityBody.setZero();
  _desiredAccelerationBody.setZero();
}

LocalPlanner::~LocalPlanner() {}

void LocalPlanner::update(robots::Robot &robot,
                          messages::HighLevelCommand &command, float dt) {

  Vec4<bool> footContacts = robot.footContacts();
  scheduler.step(dt, footContacts);

  DMat<bool> contactTable =
      scheduler.getContactTable(_stepDt, _horizonSteps, footContacts);

  DMat<float> bodyTrajectory = bodyPlanner.getDesiredBodyTrajectory(
      robot, command, _stepDt, _horizonSteps);

  footPlanner.calculateNextFootholdPositions(robot, command);

  DMat<float> footholdTable = footPlanner.calculateWorldFrameRotatedFootholds(
      robot, command, bodyTrajectory, contactTable);

  DVec<double> forces = mpc.computeContactForces(robot, contactTable,
                                                 footholdTable, bodyTrajectory);

  _mpcForces = -forces.block<12, 1>(0, 0).cast<float>();

  _desiredRpy = bodyTrajectory.block<1, 3>(0, 0);
  _desiredPositionBody = bodyTrajectory.block<1, 3>(0, 3);
  _desiredAngularVelocity = bodyTrajectory.block<1, 3>(0, 6);
  _desiredVelocityBody = bodyTrajectory.block<1, 3>(0, 9);
  _desiredAccelerationBody.setZero();

  FOR_EACH_LEG {
    bool useForceTask =
        scheduler.footInContact(LEG_ID) || scheduler.lostContact(LEG_ID);
    _footState[LEG_ID] = useForceTask;
    if (scheduler.lostContact(LEG_ID)) {
      _mpcForces(3 * LEG_ID + 2, 0) = _mpcBodyMass * 5;
    }
  }

  footPlanner.getFootDesiredPVA(robot, _desiredFootP, _desiredFootV,
                                _desiredFootA);
}

} // namespace control
} // namespace strelka
