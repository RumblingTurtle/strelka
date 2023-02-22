

#include <common/A1/constants.hpp>
#include <common/A1/kinematics.hpp>
#include <common/macros.hpp>
#include <iostream>
#include <robots/UnitreeA1.hpp>

int main() {
  using namespace strelka::robots;
  using namespace strelka;

  float q[12] = {
      -0.009244129061698914, 0.8061581254005432, -1.683815360069275,
      0.014326967298984528,  0.8092526793479919, -1.6850615739822388,
      -0.024707984179258347, 0.7794745564460754, -1.7144896984100342,
      0.027624664828181267,  0.7760597467422485, -1.7161086797714233};

  float innoControlPositions[12] = {
      0.1907501220703125,   -0.1354198157787323,  -0.26440995931625366,
      0.1916712373495102,   0.1359662115573883,   -0.2638000547885895,
      -0.1618153154850006,  -0.13619649410247803, -0.25989606976509094,
      -0.16101890802383423, 0.1366560310125351,   -0.2592769265174866};

  Vec12<float> qVec = Eigen::Map<Vec12<float>>(q, 12);
  Vec12<float> innoControlPositionsVec =
      Eigen::Map<Vec12<float>>(innoControlPositions, 12);

  FOR_EACH_LEG {

    Vec3<float> trunkToHipOffset = Eigen::Map<const Vec3<float>>(
        constants::A1::TRUNK_TO_HIP_OFFSETS + 3 * LEG_ID, 3);

    std::cout << kinematics::A1::footPositionHipFrame(
                     qVec.block(LEG_ID * 3, 0, 3, 1), LEG_ID) +
                     trunkToHipOffset -
                     innoControlPositionsVec.block(LEG_ID * 3, 0, 3, 1)
              << std::endl;
  }

  return 0;
}