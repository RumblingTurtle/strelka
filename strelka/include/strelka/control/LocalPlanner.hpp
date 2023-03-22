#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <memory>
#include <strelka/common/macros.hpp>
#include <strelka/control/BodyTrajectoryPlanner.hpp>
#include <strelka/control/FootholdPlanner.hpp>
#include <strelka/control/MPC.hpp>
#include <strelka/robots/Robot.hpp>
#include <strelka_messages/HighLevelCommand.hpp>

namespace strelka {
namespace control {

/**
 * @brief Class resposible for both foot and body planning for whole-body
 * impulse control. Additionally computes reaction forces
 *
 */
class LocalPlanner {
  std::shared_ptr<GaitScheduler> scheduler;
  std::shared_ptr<FootholdPlanner> footPlanner;
  BodyTrajectoryPlanner bodyPlanner;
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
  LocalPlanner(Gait gait, float mpcBodyMass, const Mat3<float> bodyInertia,
               float stepDt = 0.02, int horizonSteps = 15,
               float heightFilterCutoffFrequency = 30.0,
               float pitchFilterCutoffFrequency = 10);

  LocalPlanner(std::shared_ptr<FootholdPlanner> footPlanner, float mpcBodyMass,
               const Mat3<float> bodyInertia, float stepDt = 0.02,
               int horizonSteps = 15, float heightFilterCutoffFrequency = 30.0,
               float pitchFilterCutoffFrequency = 10);
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

  std::shared_ptr<FootholdPlanner> getFootPlanner();
  ~LocalPlanner();
};
} // namespace control
} // namespace strelka

#endif // LOCAL_PLANNER_H