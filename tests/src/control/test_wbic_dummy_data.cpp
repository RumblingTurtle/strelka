#include <control/A1/A1WBIC.hpp>
#include <control/WBICCommand.hpp>
#include <control/WholeBodyImpedanceController.hpp>
#include <robots/UnitreeA1.hpp>

int main() {
  strelka::WholeBodyImpedanceController controller{
      strelka::A1_DEFAULT_WBIC_PARAMS};

  strelka::WholeBodyImpedanceController::WBICOutput outs;

  strelka::robots::UnitreeA1 robot =
      strelka::robots::createDummyA1RobotWithRawState();

  a1_lcm_msgs::RobotState robotState;
  a1_lcm_msgs::WbicCommand wbicCommand;

  strelka::control::WBIC::WBICCommand command(&wbicCommand);

  for (int i = 0; i < 10; i++) {
    strelka::robots::UnitreeA1 robot(&robotState);
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
