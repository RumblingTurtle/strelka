#include <interfaces/GazeboInterface.hpp>

int main() {
  using namespace strelka;

  GazeboInterface interface;
  interface.moveToInit();
  interface.moveToStand();
  return 0;
}