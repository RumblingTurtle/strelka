#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <common/macros.hpp>
#include <control/BodyTrajectoryPlanner.hpp>
#include <control/FootholdPlanner.hpp>
#include <control/MPC.hpp>

#include <common/Robot.hpp>
#include <messages/HighLevelCommand.hpp>

namespace strelka {
namespace control {

class LocalPlanner {
  GaitScheduler scheduler;
  BodyTrajectoryPlanner bodyPlanner;
  FootholdPlanner footPlanner;
  MPC mpc;

  float _stepDt;
  int _horizonSteps;
  float _mpcBodyMass;

  float _footState[4];
  Vec12<float> _mpcForces;
  Vec12<float> _desiredFootP;
  Vec12<float> _desiredFootV;
  Vec12<float> _desiredFootA;

  Vec3<float> _desiredRpy;
  Vec3<float> _desiredPositionBody;
  Vec3<float> _desiredAngularVelocity;
  Vec3<float> _desiredVelocityBody;
  Vec3<float> _desiredAccelerationBody;

public:
  LocalPlanner(double mpcBodyMass, const Vec3<double> bodyInertia,
               float stepDt = 0.02, int horizonSteps = 15);

  float footState(int legId);
  Vec12<float> &mpcForces();
  Vec12<float> &desiredFootP();
  Vec12<float> &desiredFootV();
  Vec12<float> &desiredFootA();
  Vec3<float> &desiredRpy();
  Vec3<float> &desiredPositionBody();
  Vec3<float> &desiredAngularVelocity();
  Vec3<float> &desiredVelocityBody();
  Vec3<float> &desiredAccelerationBody();

  void update(robots::Robot &robot, messages::HighLevelCommand &command,
              float dt);

  ~LocalPlanner();
};
} // namespace control
} // namespace strelka

#endif // LOCAL_PLANNER_H