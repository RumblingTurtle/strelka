#include <control/A1/A1WBIC.hpp>
#include <control/WholeBodyImpedanceController.hpp>

int main() {
  strelka::WholeBodyImpedanceController controller{
      strelka::A1_DEFAULT_WBIC_PARAMS};

  strelka::WholeBodyImpedanceController::WBICInput ins;
  ins.bodyOrientation = {0, 0, 0, 1};
  ins.bodyPosition.setZero();
  ins.angularVelocity.setZero();
  ins.linearVelocity.setZero();
  ins.q.setZero();
  ins.dq.setZero();
  ins.pBody_RPY_des.setZero();
  ins.vBody_Ori_des.setZero();
  ins.pBody_des.setZero();
  ins.vBody_des.setZero();
  ins.aBody_des.setZero();
  ins.pFoot_des.setZero();
  ins.vFoot_des.setZero();
  ins.aFoot_des.setZero();
  ins.Fr_des_MPC.setZero();
  ins.contact_state = {1, 1, 0, 1};

  strelka::WholeBodyImpedanceController::WBICOutput outs;

  for (int i = 0; i < 10; i++) {
    std::cout << "Iter: " << i << std::endl;
    ins.bodyPosition += Vec3<float>{1, 0, 0};

    controller.update(ins, outs);

    std::cout << "q" << std::endl;
    std::cout << outs.q.transpose() << std::endl;
    std::cout << "dq" << std::endl;
    std::cout << outs.dq.transpose() << std::endl;
    std::cout << "Torques" << std::endl;
    std::cout << outs.tau.transpose() << std::endl << std::endl;
  }

  return 0;
}
