#include <common/macros.hpp>
#include <gtest/gtest.h>
#include <robots/A1/UnitreeA1.hpp>
#include <robots/A1/constants.hpp>
#include <robots/A1/interfaces/A1GazeboInterface.hpp>
#include <state_estimation/KalmanFilterObserver.hpp>

namespace {
using namespace strelka::robots;
using namespace strelka::state_estimation;

static int OBSERVER_TEST_UPDATE_COUNT = 100;

class KalmanFilterTest : public ::testing::Test {
protected:
  KalmanFilterObserver *observer;
  UnitreeA1 *robot;

  void SetUp() override {
    KalmanFilterObserver::KalmanFilterObserverParams params{
        .dt = 0.001,
        .imuPositionProcessNoise = 0.02,
        .imuVelocityProcessNoise = 0.02,
        .footPositionProcessNoise = 0.002,
        .footPositionSensorNoise = 0.001,
        .footVelocitySensorNoise = 0.1,
        .contactHeightSensorNoise = 0.001,
        .externalOdometryNoisePosition = {0.02, 0.02, 0.09},
    };

    observer = new KalmanFilterObserver{params};
    robot = new UnitreeA1(UnitreeA1::createDummyA1RobotWithRawState(
        strelka::A1::constants::STAND_ANGLES));
  }

  void TearDown() override {
    delete observer;
    delete robot;
  }
};

TEST_F(KalmanFilterTest, ConvergesOnZ) {
  float footPosZ = -robot->footPositionTrunkFrame(0)(2);

  for (int updateCount = 0; updateCount < OBSERVER_TEST_UPDATE_COUNT;
       updateCount++) {
    observer->update(*robot, false, Vec3<float>::Zero(), Vec4<float>::Zero());
  }

  // NOTE: Is this approximation correct?
  ASSERT_NEAR(observer->position()(2), footPosZ, 1e-2);
}
} // namespace
