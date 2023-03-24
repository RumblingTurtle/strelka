#include <strelka/nodes/LocalPlannerNode.hpp>
#include <strelka/robots/A1/UnitreeA1.hpp>

int main() {
  using namespace strelka::control;
  using namespace strelka::robots;

  LocalPlannerNode<UnitreeA1> planner{
      strelka::GAITS::TROT, 0.02, 15, 30.0, 10.0, false};
  planner.processLoop();
  return 0;
}