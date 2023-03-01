#include <control/BodyTrajectoryPlanner.hpp>
#include <iostream>
#include <messages/HighLevelCommand.hpp>
#include <robots/A1/UnitreeA1.hpp>
#include <robots/A1/constants.hpp>

int main() {
  using namespace strelka::control;
  using namespace strelka::robots;
  using namespace strelka::messages;

  UnitreeA1 robot = createDummyA1RobotWithStateEstimates();

  BodyTrajectoryPlanner planner{};
  float desiredVelocityX = 0.2;
  float dt = 0.001;
  int horizonSteps = 10;

  a1_lcm_msgs::HighLevelCommand highCommandMsg{
      .linearSpeed = {desiredVelocityX, 0, 0},
      .angularVelocity = {0, 0, 0},
      .footHeight = 0.12,
      .footClearance = 0.02,
      .hipOffsets = {0, 0},
      .rpy = {0, 0, 0},
      .comOffset = {0, 0},
      .bodyHeight = 0.25,
      .stop = false};

  HighLevelCommand command{&highCommandMsg};

  DMat<float> trajectory =
      planner.getDesiredBodyTrajectory(robot, command, dt, horizonSteps);

  std::cout << trajectory << std::endl;

  assert(APPROX_EQUAL(trajectory(9, 3), dt * desiredVelocityX * horizonSteps));
  return 0;
}