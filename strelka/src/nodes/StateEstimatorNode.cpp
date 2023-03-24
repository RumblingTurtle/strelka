#include <strelka/nodes/StateEstimatorNode.hpp>

namespace strelka {

namespace state_estimation {

template <class RobotClass>
StateEstimatorNode<RobotClass>::StateEstimatorNode()
    : firstRun(true), filterWarmupTime(STATE_ESTIMATOR_WARMUP_TIME),
      prevTick(-1) {
  robotStateMsg = new strelka_lcm_headers::RobotState();
  observer = new KalmanFilterObserver();

  SlowdownEstimator slowdownEstimator(lcm, constants::RAW_STATE_TOPIC_NAME);
  slowdownEstimator.estimateDts();

  DEFAULT_KALMAN_FILTER_PARAMS.dt = slowdownEstimator.getSimDt();
  observer->setParameters(DEFAULT_KALMAN_FILTER_PARAMS);
  sub = lcm.subscribe(constants::RAW_STATE_TOPIC_NAME,
                      &StateEstimatorNode::update, this);
  sub->setQueueCapacity(1);
}

template <class RobotClass> void StateEstimatorNode<RobotClass>::processLoop() {
  while (handle()) {
  }
}

template <class RobotClass> bool StateEstimatorNode<RobotClass>::handle() {
  Vec3<float> currentPosition = observer->position();
  float positionNorm = currentPosition.dot(currentPosition);

  if (0 != lcm.handle()) {
    std::cout << "StateEstimatorNode: lcm.handle() nonzero return value"
              << std::endl;
    return false;
  }

  // Check if state estimate is nan
  if (positionNorm != positionNorm) {
    std::cout << "StateEstimatorNode: NANs in position estimates" << std::endl;
    return false;
  }

  return true;
}

template <class RobotClass>
StateEstimatorNode<RobotClass>::~StateEstimatorNode() {
  delete robotStateMsg;
  delete observer;
  if (sub) {
    lcm.unsubscribe(sub);
  }
}

template <class RobotClass>
void StateEstimatorNode<RobotClass>::propagateRobotRawState(
    const strelka_lcm_headers::RobotRawState *messageIn,
    strelka_lcm_headers::RobotState *messageOut) {
  memcpy(messageOut->quaternion, messageIn->quaternion, sizeof(float) * 4);
  memcpy(messageOut->gyro, messageIn->gyro, sizeof(float) * 3);
  memcpy(messageOut->accel, messageIn->accel, sizeof(float) * 3);
  memcpy(messageOut->footForces, messageIn->footForces, sizeof(float) * 4);
  memcpy(messageOut->q, messageIn->q, sizeof(float) * 12);
  memcpy(messageOut->dq, messageIn->dq, sizeof(float) * 12);
  messageOut->tick = messageIn->tick;
}

template <class RobotClass>
void StateEstimatorNode<RobotClass>::fillStateEstimatorData(
    robots::Robot &robot, const KalmanFilterObserver *observer,
    strelka_lcm_headers::RobotState *messageOut) {

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

template <class RobotClass>
void StateEstimatorNode<RobotClass>::updateFootContactHeights(
    robots::Robot &robot) {
  if (firstRun) {
    FOR_EACH_LEG { contactHeightEstimates(LEG_ID) = robot.footRadius(); }
    previousContacts = robot.footContacts();
    firstRun = false;
    return;
  }

  Vec4<bool> footContacts = robot.footContacts();

  FOR_EACH_LEG {
    bool justHitTheGround = !previousContacts(LEG_ID) && footContacts(LEG_ID);

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

template <class RobotClass>
void StateEstimatorNode<RobotClass>::update(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const strelka_lcm_headers::RobotRawState *messageIn) {
  if (prevTick == -1) {
    prevTick = messageIn->tick;
  }

  float dt = messageIn->tick - prevTick;
  prevTick = messageIn->tick;

  robotInstance.update(messageIn);
  // TODO: Add external odometry listener
  observer->update(robotInstance, false, Vec3<float>::Zero(),
                   contactHeightEstimates);
  updateFootContactHeights(robotInstance);
  propagateRobotRawState(messageIn, robotStateMsg);
  fillStateEstimatorData(robotInstance, observer, robotStateMsg);

  if (filterWarmupTime > 0) {
    filterWarmupTime -= dt;
  } else {
    lcm.publish(constants::ROBOT_STATE_TOPIC_NAME, robotStateMsg);
  }
}
} // namespace state_estimation
#define STATE_ESTIMATOR_NODE_HEADER
#include <strelka/robots/RobotRegistry.hpp>
} // namespace strelka