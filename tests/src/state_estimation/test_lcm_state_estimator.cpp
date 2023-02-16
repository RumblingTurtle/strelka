#include <interfaces/GazeboInterface.hpp>
#include <state_estimation/A1/A1StateEstimator.hpp>

int main() {
  using namespace strelka;

  GazeboInterface interface;
  interface.moveToInit();
  interface.moveToStand();

  A1StateEstimator estimator{};
  estimator.processLoop();

  return 0;
}