#include <strelka_robots/A1/state_estimation/A1StateEstimator.hpp>

namespace strelka {

namespace state_estimation {
A1StateEstimator::A1StateEstimator()
    : firstRun(true), filterWarmupTime(STATE_ESTIMATOR_WARMUP_TIME),
      prevTick(-1) {
  robotStateMsg = new a1_lcm_msgs::RobotState();
  observer = new KalmanFilterObserver();

  SlowdownEstimator slowdownEstimator(lcm, A1::constants::RAW_STATE_TOPIC_NAME);
  slowdownEstimator.estimateDts();

  DEFAULT_KALMAN_FILTER_PARAMS.dt = slowdownEstimator.getSimDt();
  observer->setParameters(DEFAULT_KALMAN_FILTER_PARAMS);
  sub = lcm.subscribe(A1::constants::RAW_STATE_TOPIC_NAME,
                      &A1StateEstimator::update, this);
  sub->setQueueCapacity(1);
}

void A1StateEstimator::processLoop() {
  while (handle()) {
  }
}

bool A1StateEstimator::handle() { return 0 == lcm.handle(); }

A1StateEstimator::~A1StateEstimator() {
  delete robotStateMsg;
  delete observer;
  if (sub) {
    lcm.unsubscribe(sub);
  }
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

  Vec3<float> filteredVelocity =
      velocityFilter.getAverage(observer->velocityBody());

  memcpy(messageOut->position, observer->position().data(), sizeof(float) * 3);

  memcpy(messageOut->velocityBody, filteredVelocity.data(), sizeof(float) * 3);

  Vec12<float> footPosititons;
  FOR_EACH_LEG {
    footPosititons.block<3, 1>(LEG_ID * 3, 0) =
        robot.footPositionTrunkFrame(LEG_ID);
  }

  memcpy(messageOut->footPositions, footPosititons.data(), sizeof(float) * 12);
  memcpy(messageOut->jacobians, robot.footJacobians().data(),
         sizeof(float) * 36);
}

void A1StateEstimator::updateFootContactHeights(robots::UnitreeA1 &robot) {
  if (firstRun) {
    FOR_EACH_LEG { contactHeightEstimates(LEG_ID) = robot.footRadius(); }
    previousContacts = robot.footContacts();
    firstRun = false;
    return;
  }

  FOR_EACH_LEG {
    bool justHitTheGround =
        !previousContacts(LEG_ID) && robot.footContact(LEG_ID);

    float footHeightWorldFrame =
        (robot.rotateBodyToWorldFrame(robot.footPositionTrunkFrame(LEG_ID)) +
         observer->position())(2);

    bool enoughHeightDifference =
        std::abs(footHeightWorldFrame - contactHeightEstimates(LEG_ID)) >
        HEIGHT_DIFF_THRESHOLD;

    if (justHitTheGround && enoughHeightDifference) {
      contactHeightEstimates(LEG_ID) = footHeightWorldFrame;
    }
  }
  previousContacts = robot.footContacts();
}

void A1StateEstimator::update(const lcm::ReceiveBuffer *rbuf,
                              const std::string &chan,
                              const a1_lcm_msgs::RobotRawState *messageIn) {
  if (prevTick == -1) {
    prevTick = messageIn->tick;
  }

  float dt = messageIn->tick - prevTick;
  prevTick = messageIn->tick;

  robots::UnitreeA1 robot(messageIn);
  // TODO: Add external odometry listener
  observer->update(robot, false, Vec3<float>::Zero(), contactHeightEstimates);
  updateFootContactHeights(robot);
  propagateRobotRawState(messageIn, robotStateMsg);
  fillStateEstimatorData(robot, observer, robotStateMsg);

  if (filterWarmupTime > 0) {
    filterWarmupTime -= dt;
  } else {
    lcm.publish(A1::constants::ROBOT_STATE_TOPIC_NAME, robotStateMsg);
  }
}
} // namespace state_estimation
} // namespace strelka