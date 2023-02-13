#include <state_estimation/A1/A1StateEstimator.hpp>

namespace strelka {

A1StateEstimator::A1StateEstimator() {
  messageOut = new a1_lcm_msgs::RobotState();
  observer = new KalmanFilterObserver();
}

void A1StateEstimator::processLoop() {
  SlowdownEstimator slowdownEstimator(lcm);
  slowdownEstimator.estimateDt();

  A1_DEFAULT_KALMAN_FILTER_PARAMS.dt = slowdownEstimator.getSimDt();
  observer->setParameters(A1_DEFAULT_KALMAN_FILTER_PARAMS);
  sub = lcm.subscribe("raw_state", &A1StateEstimator::update, this);
  while (lcm.handle() == 0) {
  }
}

A1StateEstimator::~A1StateEstimator() {
  delete messageOut;
  delete observer;
  if (sub)
    lcm.unsubscribe(sub);
}

void A1StateEstimator::update(const lcm::ReceiveBuffer *rbuf,
                              const std::string &chan,
                              const a1_lcm_msgs::RobotRawState *messageIn) {
  KalmanFilterObserver::KalmanFilterObserverInput input;

  Eigen::Matrix<float, 36, 3> footJacobians;

  kinematics::quatToEulerMatrix(
      Eigen::Map<const Eigen::Matrix<float, 4, 1>>(messageIn->quaternion, 4),
      input.bodyToWorldMat);

  for (int legId = 0; legId < 4; legId++) {
    Eigen::Vector3f q =
        Eigen::Map<const Eigen::Vector3f>(messageIn->q + legId * 3, 3);
    Eigen::Vector3f dq =
        Eigen::Map<const Eigen::Vector3f>(messageIn->dq + legId * 3, 3);

    Eigen::Vector3f footPosition;
    Eigen::Matrix3f footJacobian;

    kinematics::analyticalLegJacobian(q, legId, footJacobian);
    kinematics::footPositionHipFrame(q, legId, footPosition);

    input.footPositionsTrunkFrame.block(legId, 0, 1, 3) =
        footPosition.transpose() +
        Eigen::Map<const Eigen::Vector3f>(TRUNK_TO_HIP_OFFSETS + 3 * legId, 3)
            .transpose();

    input.footVelocitiesTrunkFrame.block(legId, 0, 1, 3) =
        (footJacobian * dq).transpose();
    footJacobians.block(legId * 3, 0, 3, 3) = footJacobian;
    input.footContacts(legId) =
        messageIn->footForces[legId] > FOOT_FORCE_THRESHOLD;
    input.footContactHeights(legId) = 0;
  }

  input.gyroscope = Eigen::Map<const Eigen::Vector3f>(messageIn->gyro, 3);

  input.accelerometer =
      input.bodyToWorldMat *
          Eigen::Map<const Eigen::Vector3f>(messageIn->accel, 3) +
      GRAVITY_CONSTANT;

  input.externalOdometryPosition.setZero();
  input.useExternalOdometry = false;

  KalmanFilterObserver::KalmanFilterObserverOutput output;
  observer->update(input, output);

  memcpy(messageOut->quaternion, messageIn->quaternion, sizeof(float) * 4);
  memcpy(messageOut->gyro, messageIn->gyro, sizeof(float) * 3);
  memcpy(messageOut->accel, messageIn->accel, sizeof(float) * 3);
  memcpy(messageOut->footForces, messageIn->footForces, sizeof(float) * 4);
  memcpy(messageOut->q, messageIn->q, sizeof(float) * 12);
  memcpy(messageOut->dq, messageIn->dq, sizeof(float) * 12);

  memcpy(messageOut->position, output.position.data(), sizeof(float) * 3);
  memcpy(messageOut->velocityBody, output.velocityBody.data(),
         sizeof(float) * 3);
  memcpy(messageOut->footPositions, input.footPositionsTrunkFrame.data(),
         sizeof(float) * 12);

  memcpy(messageOut->jacobians, footJacobians.data(), sizeof(float) * 36);

  messageOut->tick = messageIn->tick;

  lcm.publish("robot_state", messageOut);
}
} // namespace strelka