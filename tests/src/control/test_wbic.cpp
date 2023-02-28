#include <control/A1/A1WBIC.hpp>
#include <control/A1/A1WbicDynamics.hpp>
#include <control/WholeBodyImpedanceController.hpp>
#include <messages/WBICCommand.hpp>
#include <robots/UnitreeA1.hpp>

int main() {
  using namespace strelka::control;
  using namespace strelka::messages;
  using namespace strelka::robots;

  WholeBodyImpedanceController controller{buildA1<float>().buildModel(),
                                          strelka::A1_DEFAULT_WBIC_PARAMS};

  WholeBodyImpedanceController::WBICOutput outs;

  UnitreeA1 robot = createDummyA1RobotWithRawState();

  a1_lcm_msgs::RobotState robotState;
  a1_lcm_msgs::WbicCommand wbicCommand;

  WBICCommand command(&wbicCommand);

  for (int i = 0; i < 10; i++) {
    std::cout << "Iter: " << i << std::endl;
    wbicCommand.pBody[0] += 1;
    controller.update(robot, command, outs);

    std::cout << "q" << std::endl;
    std::cout << outs.q.transpose() << std::endl;
    std::cout << "dq" << std::endl;
    std::cout << outs.dq.transpose() << std::endl;
    std::cout << "Torques" << std::endl;
    std::cout << outs.tau.transpose() << std::endl << std::endl;
  }

  return 0;
}
