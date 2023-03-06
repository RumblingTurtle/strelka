#include <common/macros.hpp>
#include <gtest/gtest.h>
#include <robots/A1/interfaces/A1GazeboInterface.hpp>

namespace {
using namespace strelka::interfaces;

class A1GazeboInterfaceParametrizedTestFixture
    : public ::testing::TestWithParam<Vec3<float>> {
protected:
  A1GazeboInterface interface;
};

TEST_P(A1GazeboInterfaceParametrizedTestFixture, SetDesiredAngles) {

  try {
    interface.moveToInit();
  } catch (const RobotStateTopicDoesNotExist &ex) {
    GTEST_SKIP()
        << "A1 Gazebo topic is not publishing. Default value of the topic name "
           "is "
        << strelka::A1::constants::RAW_STATE_TOPIC_NAME;
  }

  sleep(1);
  const Vec3<float> desiredAngles = GetParam();
  interface.moveTo(desiredAngles, 3.0);
  sleep(1);
  Vec12<float> actualAngles = interface.getAngles();

  FOR_EACH_LEG {
    Vec3<float> qLeg = actualAngles.block<3, 1>(3 * LEG_ID, 0);
    for (int dim = 0; dim < 3; dim++) {
      float desired = std::abs(desiredAngles(dim));
      float actual = std::abs(qLeg(dim));
      // 5 degree error is bearable
      EXPECT_NEAR(desired, actual, 1e-1);
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    A1GazeboInterfaceTests, A1GazeboInterfaceParametrizedTestFixture,
    ::testing::Values(strelka::A1::constants::STAND_ANGLES,
                      strelka::A1::constants::INIT_ANGLES));
} // namespace