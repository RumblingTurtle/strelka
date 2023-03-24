#include <strelka/nodes/WBICNode.hpp>
#include <strelka/robots/A1/UnitreeA1.hpp>

int main() {
  using namespace strelka::robots;
  using namespace strelka::control;

  WBICNode<UnitreeA1> wbicController{};
  wbicController.processLoop();
  return 0;
}