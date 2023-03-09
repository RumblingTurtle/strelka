#include <strelka_robots/A1/interfaces/A1GazeboInterface.hpp>
#include <strelka_robots/A1/state_estimation/A1StateEstimator.hpp>

int main() {
  using namespace strelka::interfaces;
  using namespace strelka::state_estimation;

  A1GazeboInterface interface;
  interface.moveToInit();
  interface.moveToStand();

  A1StateEstimator estimator{};
  estimator.processLoop();

  return 0;
}