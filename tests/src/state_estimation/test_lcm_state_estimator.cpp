#include <robots/A1/interfaces/A1GazeboInterface.hpp>
#include <robots/A1/state_estimation/A1StateEstimator.hpp>

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