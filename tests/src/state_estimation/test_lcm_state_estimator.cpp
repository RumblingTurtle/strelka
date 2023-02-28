#include <interfaces/A1/A1GazeboInterface.hpp>
#include <state_estimation/A1/A1StateEstimator.hpp>

int main() {
  using namespace strelka::A1;
  using namespace strelka;

  A1GazeboInterface interface;
  interface.moveToInit();
  interface.moveToStand();

  A1StateEstimator estimator{};
  estimator.processLoop();

  return 0;
}