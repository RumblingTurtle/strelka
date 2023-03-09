#include <strelka_robots/A1/control/A1LocalPlanner.hpp>

int main() {
  using namespace strelka::control;
  A1LocalPlanner planner{strelka::GAITS::TROT};
  planner.processLoop();
  return 0;
}