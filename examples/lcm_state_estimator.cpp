#include <strelka/nodes/MoveToInterface.hpp>
#include <strelka/nodes/StateEstimatorNode.hpp>
#include <strelka/robots/A1/UnitreeA1.hpp>

int main() {
  using namespace strelka::interfaces;
  using namespace strelka::robots;
  using namespace strelka::state_estimation;

  MoveToInterface<UnitreeA1> interface {};

  interface.moveToInit();
  interface.moveToStand();

  StateEstimatorNode<UnitreeA1> estimator{};
  estimator.processLoop();

  return 0;
}