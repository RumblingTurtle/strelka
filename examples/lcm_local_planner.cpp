#include <strelka/nodes/LocalPlannerNode.hpp>
#include <strelka/robots/A1/UnitreeA1.hpp>

int main() {
  using namespace strelka::control;
  using namespace strelka::robots;

  // Used only to take mass and inertia constants
  UnitreeA1 robot = UnitreeA1::createDummyA1RobotWithRawState();

  LocalPlannerNode<UnitreeA1> planner{strelka::GAITS::TROT,
                                      robot.robotMass(),
                                      robot.rotationalInertia(),
                                      0.02,
                                      15,
                                      30.0,
                                      10.0};
  planner.processLoop();
  return 0;
}