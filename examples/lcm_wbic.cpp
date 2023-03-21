#include <strelka_robots/A1/UnitreeA1.hpp>
#include <strelka_robots/WBICNode.hpp>

int main() {
  using namespace strelka::robots;
  using namespace strelka::control;

  UnitreeA1 robot = UnitreeA1::createDummyA1RobotWithStateEstimates();
  WBICNode<UnitreeA1> wbicController{robot, DEFAULT_WBIC_PARAMS};
  wbicController.processLoop();
  return 0;
}