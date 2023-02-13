#ifndef SLOWDOWN_ESTIMATOR_H
#define SLOWDOWN_ESTIMATOR_H

#include <a1_lcm_msgs/RobotRawState.hpp>
#include <chrono>
#include <common/typedefs.hpp>
#include <exception>
#include <lcm/lcm-cpp.hpp>

#define MAX_MEASUREMENT_COUNT 1000
namespace strelka {

class InvalidSlowdownEstimate : std::exception {
  const char *what() {
    return "Slownown estimator haven't made any measurements "
           "or encountered an "
           "error during estimation";
  }
};

class SlowdownEstimator {
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
} // namespace strelka

#endif