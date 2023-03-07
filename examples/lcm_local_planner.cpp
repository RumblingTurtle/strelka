#include <robots/A1/control/A1LocalPlanner.hpp>

int main() {
  using namespace strelka::control;
  A1LocalPlanner planner{strelka::GAITS::FLYTROT};
  planner.processLoop();
  return 0;
}