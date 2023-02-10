#include <A1/constants.hpp>
#include <interfaces/GazeboInterface.hpp>
#include <iostream>

int main() {
  strelka::GazeboInterface interface;
  interface.moveToInit();
  interface.moveToStand();
  return 0;
}