#include <strelka_robots/A1/control/A1WBIC.hpp>

int main() {
  using namespace strelka;
  using namespace strelka::control;

  A1WBIC wbicController{DEFAULT_WBIC_PARAMS};
  wbicController.processLoop();
  return 0;
}