#include <strelka_robots/A1/UnitreeA1.hpp>
#include <strelka_robots/LocalPlannerNode.hpp>

int main() {
  using namespace strelka::control;
  using namespace strelka::robots;

  // Used only to take mass and inertia constants
  UnitreeA1 robot = UnitreeA1::createDummyA1RobotWithRawState();

  LocalPlannerNode<UnitreeA1> planner{strelka::GAITS::TROT, robot.robotMass(),
                                      robot.rotationalInertia()};
  planner.processLoop();
  return 0;
}