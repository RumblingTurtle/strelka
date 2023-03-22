#include <strelka/nodes/MoveToInterface.hpp>
#include <strelka/nodes/StateEstimatorNode.hpp>
#include <strelka/robots/A1/UnitreeA1.hpp>

int main() {
  using namespace strelka::interfaces;
  using namespace strelka::robots;
  using namespace strelka::state_estimation;
  UnitreeA1 robot = UnitreeA1::createDummyA1RobotWithRawState();

  MoveToInterface<UnitreeA1> interface {
    robot
  };

  interface.moveToInit();
  interface.moveToStand();

  StateEstimatorNode<UnitreeA1> estimator{};
  estimator.processLoop();

  return 0;
}