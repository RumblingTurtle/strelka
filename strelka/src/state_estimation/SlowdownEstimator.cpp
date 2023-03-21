#include <strelka/state_estimation/SlowdownEstimator.hpp>

namespace strelka {

namespace state_estimation {
void SlowdownEstimator::updateDt(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const strelka_lcm_headers::RobotRawState *messageIn) {
  if (prevTickSim == -1) {
    prevTickSim = messageIn->tick;
    prevTickReal = getWallTime();
    return;
  }

  dtEstimateSim += (messageIn->tick - prevTickSim);
  dtEstimateReal += timePointDiffInSeconds(getWallTime(), prevTickReal);

  measurementCount++;
  prevTickSim = messageIn->tick;
  prevTickReal = getWallTime();
}

void SlowdownEstimator::reset() {
  prevTickSim = -1;
  dtEstimateSim = 0.0;
  dtEstimateReal = 0.0;
  measurementCount = 0;
}

SlowdownEstimator::SlowdownEstimator(lcm::LCM &lcm, const char *topicName)
    : lcm(lcm), topicName(topicName) {
  reset();
}

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

void SlowdownEstimator::estimateDts() {
  reset();
  // NOTE: move to strelka_robots?
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
} // namespace state_estimation
} // namespace strelka
