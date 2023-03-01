#ifndef SLOWDOWN_ESTIMATOR_H
#define SLOWDOWN_ESTIMATOR_H

#include <chrono>
#include <exception>
#include <lcm/lcm-cpp.hpp>
#include <messages/a1_lcm_msgs/RobotRawState.hpp>

namespace strelka {

namespace state_estimation {
class InvalidSlowdownEstimate : std::exception {
  const char *what() {
    return "Slownown estimator haven't made any measurements "
           "or encountered an "
           "error during estimation";
  }
};

class SlowdownEstimator {
  typedef std::chrono::time_point<std::chrono::high_resolution_clock>
      chrono_time_point;

  static const int MAX_MEASUREMENT_COUNT = 1000;
  int measurementCount;

  float dtEstimateSim;
  float prevTickSim;

  float dtEstimateReal;
  chrono_time_point prevTickReal;
  lcm::LCM &lcm;

  void updateDt(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                const a1_lcm_msgs::RobotRawState *messageIn);

  void reset();

public:
  SlowdownEstimator(lcm::LCM &lcm);

  bool estimateIsValid();

  float getRealtimeDt();

  float getSimDt();

  void estimateDt();
};
} // namespace state_estimation
} // namespace strelka

#endif