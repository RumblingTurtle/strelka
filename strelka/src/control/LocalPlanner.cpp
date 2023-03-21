#include <strelka/control/LocalPlanner.hpp>

namespace strelka {
namespace control {

LocalPlanner::LocalPlanner(Gait gait, float mpcBodyMass,
                           const Mat3<float> bodyInertia, float stepDt,
                           int horizonSteps)
    : scheduler(std::make_shared<GaitScheduler>(gait)), bodyPlanner(),
      _stepDt(stepDt), _horizonSteps(horizonSteps), _mpcBodyMass(mpcBodyMass),
      mpc(mpcBodyMass, bodyInertia, horizonSteps, stepDt) {
  assert(stepDt > 0.0f);
  assert(horizonSteps >= 1);
  footPlanner = std::make_shared<FootholdPlanner>(scheduler);
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

LocalPlanner::LocalPlanner(std::shared_ptr<FootholdPlanner> footholdPlanner,
                           float mpcBodyMass, const Mat3<float> bodyInertia,
                           float stepDt, int horizonSteps)
    : scheduler(footholdPlanner->gaitScheduler()), bodyPlanner(),
      footPlanner(footholdPlanner), _stepDt(stepDt),
      _horizonSteps(horizonSteps), _mpcBodyMass(mpcBodyMass),
      mpc(mpcBodyMass, bodyInertia, horizonSteps, stepDt) {
  assert(stepDt > 0.0f);
  assert(horizonSteps >= 1);

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

  command.setIgnoreDesiredBodyVelocity(scheduler->isCurrentGaitStationary());

  Vec4<bool> footContacts = robot.footContacts();
  scheduler->step(dt, footContacts);

  DMat<bool> contactTable =
      scheduler->getContactTable(_stepDt, _horizonSteps, footContacts);

  DMat<float> bodyTrajectory = bodyPlanner.getDesiredBodyTrajectory(
      robot, command, _stepDt, _horizonSteps);

  footPlanner->calculateNextFootholdPositions(robot, command);

  DMat<float> footholdTable = footPlanner->calculateWorldFrameRotatedFootholds(
      robot, command, bodyTrajectory, contactTable);

  DVec<float> forces = mpc.computeContactForces(robot, contactTable,
                                                footholdTable, bodyTrajectory);

  _mpcForces = -forces.block<12, 1>(0, 0);

  _desiredRpy = bodyTrajectory.block<1, 3>(0, 0);
  _desiredPositionBody = bodyTrajectory.block<1, 3>(0, 3);
  _desiredAngularVelocity = bodyTrajectory.block<1, 3>(0, 6);
  _desiredVelocityBody = bodyTrajectory.block<1, 3>(0, 9);
  _desiredAccelerationBody.setZero();

  FOR_EACH_LEG {
    bool useForceTask = scheduler->legInStance(LEG_ID) ||
                        scheduler->legInEarlyContact(LEG_ID) ||
                        scheduler->legLostContact(LEG_ID);
    _footState[LEG_ID] = useForceTask;
    if (scheduler->legLostContact(LEG_ID)) {
      _mpcForces(3 * LEG_ID + 2, 0) = _mpcBodyMass * 5;
    }
  }

  footPlanner->getFootDesiredPVA(robot, command, _desiredFootP, _desiredFootV,
                                 _desiredFootA);
}

bool LocalPlanner::footState(int legId) { return _footState[legId]; }

Vec12<float> &LocalPlanner::mpcForces() { return _mpcForces; }

Vec12<float> &LocalPlanner::desiredFootPositions() { return _desiredFootP; }

Vec12<float> &LocalPlanner::desiredFootVelocities() { return _desiredFootV; }

Vec12<float> &LocalPlanner::desiredFootAccelerations() { return _desiredFootA; }

Vec3<float> &LocalPlanner::desiredRpy() { return _desiredRpy; }

Vec3<float> &LocalPlanner::desiredPositionBody() {
  return _desiredPositionBody;
}
Vec3<float> &LocalPlanner::desiredAngularVelocity() {
  return _desiredAngularVelocity;
}
Vec3<float> &LocalPlanner::desiredVelocityBody() {
  return _desiredVelocityBody;
}
Vec3<float> &LocalPlanner::desiredAccelerationBody() {
  return _desiredAccelerationBody;
}

std::shared_ptr<FootholdPlanner> LocalPlanner::getFootPlanner() {
  return footPlanner;
}
} // namespace control
} // namespace strelka
