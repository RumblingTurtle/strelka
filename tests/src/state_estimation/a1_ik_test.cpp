

#include <gtest/gtest.h>
#include <strelka_common/macros.hpp>
#include <strelka_robots/A1/UnitreeA1.hpp>
#include <strelka_robots/A1/constants.hpp>
#include <strelka_robots/A1/kinematics.hpp>

namespace {
using namespace strelka::robots;
using namespace strelka;

/**
 * Stand positions taken from simulation and evaluated using
 * functions initially written in Python.
 */
static float q[12] = {
    -0.009244129061698914, 0.8061581254005432, -1.683815360069275,
    0.014326967298984528,  0.8092526793479919, -1.6850615739822388,
    -0.024707984179258347, 0.7794745564460754, -1.7144896984100342,
    0.027624664828181267,  0.7760597467422485, -1.7161086797714233};

static float trunkFrameFootPositionsTable[12] = {
    0.1907501220703125,   -0.1354198157787323,  -0.26440995931625366,
    0.1916712373495102,   0.1359662115573883,   -0.2638000547885895,
    -0.1618153154850006,  -0.13619649410247803, -0.25989606976509094,
    -0.16101890802383423, 0.1366560310125351,   -0.2592769265174866};

TEST(TestA1Kinematics, TrunkFramePositions) {
  FOR_EACH_LEG {
    Vec3<float> qVec = Eigen::Map<Vec3<float>>(q + 3 * LEG_ID, 3);
    Vec3<float> footPositionHipFrameTable =
        Eigen::Map<Vec3<float>>(trunkFrameFootPositionsTable + 3 * LEG_ID, 3) -
        Eigen::Map<const Vec3<float>>(
            A1::constants::TRUNK_TO_HIP_OFFSETS + 3 * LEG_ID, 3);

    Vec3<float> footPositionHipFrame =
        A1::kinematics::footPositionHipFrame(qVec, LEG_ID);

    Vec3<float> posDiff = footPositionHipFrame - footPositionHipFrameTable;

    ASSERT_NEAR(posDiff.norm(), 0.0, 1e-2);
  }
}

} // namespace