#include <control/A1/A1WBIC.hpp>

int main() {
  strelka::A1WBIC wbicController{strelka::A1_STAND_WBIC_PARAMS};
  wbicController.processLoop();
  return 0;
}