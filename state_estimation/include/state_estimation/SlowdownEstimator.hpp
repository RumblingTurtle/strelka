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

/**
 * @brief Compares wall time with RobotRawState messages' tick
 * parameter to estimate the amount of simulation slowdown
 *
 */
class SlowdownEstimator {
  typedef std::chrono::time_point<std::chrono::high_resolution_clock>
      ChronoTimePoint;

  static const int MAX_MEASUREMENT_COUNT = 1000;
  int measurementCount;

  float dtEstimateSim;
  float prevTickSim;

  float dtEstimateReal;
  ChronoTimePoint prevTickReal;
  lcm::LCM &lcm;

  void updateDt(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                const a1_lcm_msgs::RobotRawState *messageIn);

  void reset();

public:
  SlowdownEstimator(lcm::LCM &lcm);

  /**
   * @return false if o measurements are made
   */
  bool estimateIsValid();

  /**
   * @brief Get the dt between the RobotRawState messages based on wall time
   *
   * @return float delta time estimate
   */
  float getRealtimeDt();

  /**
   * @brief Get the dt between the RobotRawState messages based on
   * RobotRawState's tick
   *
   * @return float delta time estimate
   */
  float getSimDt();

  /**
   * @brief Starts taking measurements of wall and sim time. Results are
   * accessible through getRealtimeDt and getSimDt
   *
   */
  void estimateDts();
};
} // namespace state_estimation
} // namespace strelka

#endif