#include <robots/A1/interfaces/A1GazeboInterface.hpp>

int main() {
  using namespace strelka::A1;

  A1GazeboInterface interface;
  interface.moveToInit();
  interface.moveToStand();
  return 0;
}