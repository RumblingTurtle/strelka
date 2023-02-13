#include <state_estimation/SlowdownEstimator.hpp>

namespace strelka {

void SlowdownEstimator::updateDt(const lcm::ReceiveBuffer *rbuf,
                                 const std::string &chan,
                                 const a1_lcm_msgs::RobotRawState *messageIn) {
  if (prevTickSim == -1) {
    prevTickSim = messageIn->tick;
    prevTickReal = std::chrono::high_resolution_clock::now();
    return;
  }

  dtEstimateSim += (messageIn->tick - prevTickSim);
  dtEstimateReal +=
      std::chrono::duration_cast<std::chrono::duration<float>>(
          std::chrono::high_resolution_clock::now() - prevTickReal)
          .count();

  measurementCount++;
  prevTickSim = messageIn->tick;
  prevTickReal = std::chrono::high_resolution_clock::now();
}

void SlowdownEstimator::reset() {
  prevTickSim = -1;
  dtEstimateSim = 0.0;
  dtEstimateReal = 0.0;
  measurementCount = 0;
}

SlowdownEstimator::SlowdownEstimator(lcm::LCM &lcm) : lcm(lcm) { reset(); }

bool SlowdownEstimator::estimateIsValid() { return measurementCount > 0; }

float SlowdownEstimator::getRealtimeDt() {
  if (estimateIsValid()) {
    return dtEstimateReal;
  }
  throw InvalidSlowdownEstimate();
}

float SlowdownEstimator::getSimDt() {
  if (estimateIsValid()) {
    return dtEstimateSim;
  }
  throw InvalidSlowdownEstimate();
}

void SlowdownEstimator::estimateDt() {
  reset();
  lcm::Subscription *estimateSub =
      lcm.subscribe("raw_state", &SlowdownEstimator::updateDt, this);

  for (int i = 0; i < MAX_MEASUREMENT_COUNT; i++) {
    if (lcm.handle() != 0)
      break;
  }

  if (estimateIsValid()) {
    dtEstimateSim /= measurementCount;
    dtEstimateReal /= measurementCount;
  } else {
    reset();
  }

  lcm.unsubscribe(estimateSub);
}
} // namespace strelka
