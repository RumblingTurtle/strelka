#include <robots/A1/control/A1LocalPlanner.hpp>

int main() {
  using namespace strelka::control;
  A1LocalPlanner planner{};
  planner.processLoop();
  return 0;
}