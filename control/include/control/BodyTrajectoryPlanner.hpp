#ifndef BODY_TRAJECTORY_PLANNER_H
#define BODY_TRAJECTORY_PLANNER_H
#include <common/Robot.hpp>
#include <common/constants.hpp>
#include <common/macros.hpp>
#include <common/rotation.hpp>
#include <common/typedefs.hpp>
#include <filters/FirstOrderLPF.hpp>
#include <messages/HighLevelCommand.hpp>

namespace strelka {
namespace control {
/**
 * @brief Desired trajectory generator for robot body.
 *
 */
class BodyTrajectoryPlanner {

  Mat43<float> prevContactPosWorld;
  Mat43<float> prevContactPosBody;
  filters::FirstOrderLPF<float> heightFilter;
  filters::FirstOrderLPF<float> pitchFilter;

  bool firstRun;

public:
  BodyTrajectoryPlanner();

  /**
   * @brief Generate desired body trajectory on the given horizon and step
   * length in seconds
   *
   * State values per each row are as follows:
   * 0-2 Roll pitch yaw
   * 3-5 Body position in odometry frame
   * 6-8 Angular velocity
   * 9-11 Linear velocity
   * 12 Gravity constant
   *
   * @param robot Robot object implementing Robot interface
   * @param command High level command object
   * @param dt Time difference between the steps
   * @param horizonSteps The amount of steps to generate
   * @return DMat<float> horizonStepsx13 matrix of the states
   */
  DMat<float> getDesiredBodyTrajectory(robots::Robot &robot,
                                       messages::HighLevelCommand &command,
                                       float dt, int horizonSteps);
};
} // namespace control
} // namespace strelka

#endif // BODY_TRAJECTORY_PLANNER_H