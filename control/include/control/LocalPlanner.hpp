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

/**
 * @brief Class resposible for both foot and body planning for whole-body
 * impulse control. Additionally computes reaction forces
 *
 */
class LocalPlanner {
  GaitScheduler scheduler;
  BodyTrajectoryPlanner bodyPlanner;
  FootholdPlanner footPlanner;
  MPC mpc;

  float _stepDt;
  int _horizonSteps;
  float _mpcBodyMass;

  bool _footState[4];
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

  /**
   * @brief Update desired trajectories and forces according to high level
   * command and robot state
   *
   * @param robot Class implementing Robot interface
   * @param command High level command
   * @param dt The amount of time passed after the last update
   */
  void update(robots::Robot &robot, messages::HighLevelCommand &command,
              float dt);

  /**
   * @brief Returns true if WBIC should use forces as a task supplied by the
   * local planner for given legId. Foot positions, velocities and accelerations
   * are used otherwise
   */
  bool footState(int legId);

  Vec12<float> &mpcForces();
  Vec12<float> &desiredFootPositions();
  Vec12<float> &desiredFootVelocities();
  Vec12<float> &desiredFootAccelerations();
  Vec3<float> &desiredRpy();
  Vec3<float> &desiredPositionBody();
  Vec3<float> &desiredAngularVelocity();
  Vec3<float> &desiredVelocityBody();
  Vec3<float> &desiredAccelerationBody();

  ~LocalPlanner();
};
} // namespace control
} // namespace strelka

#endif // LOCAL_PLANNER_H