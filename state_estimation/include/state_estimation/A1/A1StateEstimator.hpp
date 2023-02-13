#include <A1/constants.hpp>
#include <A1/kinematics.hpp>
#include <a1_lcm_msgs/RobotRawState.hpp>
#include <a1_lcm_msgs/RobotState.hpp>
#include <chrono>
#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <state_estimation/KalmanFilterObserver.hpp>
#define MAX_TICK_MEASUREMENTS 1000

namespace strelka {
KalmanFilterObserver::KalmanFilterObserverParams A1_KALMAN_FILTER_PARAMS = {
    .dt = 0.001,
    .imuPositionProcessNoise = 0.02,
    .imuVelocityProcessNoise = 0.02,
    .footPositionProcessNoise = 0.002,
    .footPositionSensorNoise = 0.001,
    .footVelocitySensorNoise = 0.1,
    .contactHeightSensorNoise = 0.001,
    .externalOdometryNoisePosition = {0.02, 0.02, 0.09},
};

class A1StateEstimator {

  /*
  LCM callbacks
  */

  lcm::LCM lcm;
  std::unique_ptr<KalmanFilterObserver> observer;
  a1_lcm_msgs::RobotState *messageOut;
  lcm::Subscription *sub;

  float dt;
  float dtEstimateSim;
  float prevTickSim;

  float dtEstimateReal;
  std::chrono::time_point<std::chrono::high_resolution_clock> prevTickReal;

  void estimateDt() {
    lcm::Subscription *estimateSub =
        lcm.subscribe("raw_state", &A1StateEstimator::updateDt, this);
    for (int i = 0; i < MAX_TICK_MEASUREMENTS; i++) {
      lcm.handle();
    }
    lcm.unsubscribe(estimateSub);
    print("Estimated dt in sim: ");
    print(dtEstimateSim);
    print("Estimated dt from wall time: ");
    print(dtEstimateReal);
  }

  void updateDt(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                const a1_lcm_msgs::RobotRawState *messageIn) {
    if (prevTickSim == -1) {
      prevTickSim = messageIn->tick;
      prevTickReal = std::chrono::high_resolution_clock::now();
      return;
    }
    dtEstimateSim += (messageIn->tick - prevTickSim) / MAX_TICK_MEASUREMENTS;
    dtEstimateReal +=
        std::chrono::duration_cast<std::chrono::duration<float>>(
            std::chrono::high_resolution_clock::now() - prevTickReal)
            .count() /
        MAX_TICK_MEASUREMENTS;

    prevTickSim = messageIn->tick;
    prevTickReal = std::chrono::high_resolution_clock::now();
  }

public:
  A1StateEstimator()
      : dt(0.001), prevTickSim(-1), dtEstimateSim(0.0), dtEstimateReal(0.0) {
    messageOut = new a1_lcm_msgs::RobotState();
  }

  void processLoop() {
    estimateDt();
    A1_KALMAN_FILTER_PARAMS.dt = dtEstimateReal;
    observer = std::make_unique<KalmanFilterObserver>(A1_KALMAN_FILTER_PARAMS);
    sub = lcm.subscribe("raw_state", &A1StateEstimator::update, this);
    while (lcm.handle() == 0) {
    }
  }

  ~A1StateEstimator() {
    delete messageOut;
    lcm.unsubscribe(sub);
  }

  void update(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
              const a1_lcm_msgs::RobotRawState *messageIn) {
    KalmanFilterObserver::KalmanFilterObserverInput input;

    Eigen::Matrix<float, 36, 3> footJacobians;

    quatToEulerMatrix(
        Eigen::Map<const Eigen::Matrix<float, 4, 1>>(messageIn->quaternion, 4),
        input.bodyToWorldMat);

    for (int legId = 0; legId < 4; legId++) {
      Eigen::Vector3f q =
          Eigen::Map<const Eigen::Vector3f>(messageIn->q + legId * 3, 3);
      Eigen::Vector3f dq =
          Eigen::Map<const Eigen::Vector3f>(messageIn->dq + legId * 3, 3);

      Eigen::Vector3f footPosition;
      Eigen::Matrix3f footJacobian;

      analyticalLegJacobian(q, legId, footJacobian);
      footPositionHipFrame(q, legId, footPosition);

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
};
} // namespace strelka