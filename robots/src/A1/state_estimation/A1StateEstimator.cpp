#include <robots/A1/state_estimation/A1StateEstimator.hpp>

namespace strelka {

namespace state_estimation {
A1StateEstimator::A1StateEstimator() {
  robotStateMsg = new a1_lcm_msgs::RobotState();
  observer = new KalmanFilterObserver();
}

void A1StateEstimator::processLoop() {
  SlowdownEstimator slowdownEstimator(lcm);
  slowdownEstimator.estimateDts();

  DEFAULT_KALMAN_FILTER_PARAMS.dt = slowdownEstimator.getSimDt();
  observer->setParameters(DEFAULT_KALMAN_FILTER_PARAMS);
  sub = lcm.subscribe("raw_state", &A1StateEstimator::update, this);
  sub->setQueueCapacity(1);
  while (lcm.handle() == 0) {
  }
}

A1StateEstimator::~A1StateEstimator() {
  delete robotStateMsg;
  delete observer;
  if (sub)
    lcm.unsubscribe(sub);
}

void A1StateEstimator::propagateRobotRawState(
    const a1_lcm_msgs::RobotRawState *messageIn,
    a1_lcm_msgs::RobotState *messageOut) {
  memcpy(messageOut->quaternion, messageIn->quaternion, sizeof(float) * 4);
  memcpy(messageOut->gyro, messageIn->gyro, sizeof(float) * 3);
  memcpy(messageOut->accel, messageIn->accel, sizeof(float) * 3);
  memcpy(messageOut->footForces, messageIn->footForces, sizeof(float) * 4);
  memcpy(messageOut->q, messageIn->q, sizeof(float) * 12);
  memcpy(messageOut->dq, messageIn->dq, sizeof(float) * 12);
  messageOut->tick = messageIn->tick;
}

void A1StateEstimator::fillStateEstimatorData(
    robots::UnitreeA1 &robot, const KalmanFilterObserver *observer,
    a1_lcm_msgs::RobotState *messageOut) {

  memcpy(messageOut->position, observer->position().data(), sizeof(float) * 3);
  memcpy(messageOut->velocityBody, observer->velocityBody().data(),
         sizeof(float) * 3);

  Vec12<float> footPosititons;
  FOR_EACH_LEG {
    footPosititons.block<3, 1>(LEG_ID * 3, 0) =
        robot.footPositionTrunkFrame(LEG_ID);
  }

  memcpy(messageOut->footPositions, footPosititons.data(), sizeof(float) * 12);
  memcpy(messageOut->jacobians, robot.footJacobians().data(),
         sizeof(float) * 36);
}

void A1StateEstimator::update(const lcm::ReceiveBuffer *rbuf,
                              const std::string &chan,
                              const a1_lcm_msgs::RobotRawState *messageIn) {

  robots::UnitreeA1 robot(messageIn);

  // TODO: Add external odometry listener
  observer->update(robot, false, Vec3<float>::Zero());

  propagateRobotRawState(messageIn, robotStateMsg);
  fillStateEstimatorData(robot, observer, robotStateMsg);

  lcm.publish("robot_state", robotStateMsg);
}
} // namespace state_estimation
} // namespace strelka