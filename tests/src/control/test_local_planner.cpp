#include <control/LocalPlanner.hpp>

int main() {
  using namespace strelka::control;
  LocalPlanner planner{};
  planner.processLoop();
  return 0;
}