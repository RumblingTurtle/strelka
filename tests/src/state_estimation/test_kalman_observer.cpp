#include <common/A1/constants.hpp>
#include <common/macros.hpp>
#include <interfaces/GazeboInterface.hpp>
#include <iostream>
#include <robots/UnitreeA1.hpp>
#include <state_estimation/KalmanFilterObserver.hpp>

using strelka::KalmanFilterObserver;

using namespace strelka::robots;

int main() {
  using namespace strelka::robots;
  using namespace strelka;

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

  KalmanFilterObserver observer(params);
  UnitreeA1 robot = createDummyA1RobotWithRawState();

  for (int updateCount = 0; updateCount < 10; updateCount++) {
    observer.update(robot, false, Vec3<float>::Zero());
  }

  std::cout << observer.position() << std::endl;

  assert(APPROX_EQUAL(observer.position()(2), 0.308643));
  return 0;
}