#include <control/A1/A1WBIC.hpp>

int main() {
  using namespace strelka;
  using namespace strelka::control;

  A1WBIC wbicController{DEFAULT_WBIC_PARAMS};
  wbicController.processLoop();
  return 0;
}