#ifndef SLOWDOWN_ESTIMATOR_H
#define SLOWDOWN_ESTIMATOR_H

#include <exception>
#include <lcm/lcm-cpp.hpp>
#include <strelka_common/typedefs.hpp>
#include <strelka_common/utilities.hpp>
#include <strelka_messages/a1_lcm_msgs/RobotRawState.hpp>

namespace strelka {

namespace state_estimation {
class InvalidSlowdownEstimate : public std::exception {
public:
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

  static const int MAX_MEASUREMENT_COUNT = 1000;
  int measurementCount;

  float dtEstimateSim;
  float prevTickSim;

  float dtEstimateReal;
  ChronoTimePoint prevTickReal;
  lcm::LCM &lcm;
  const char *topicName;

  void updateDt(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                const a1_lcm_msgs::RobotRawState *messageIn);

  void reset();

public:
  SlowdownEstimator(lcm::LCM &lcm, const char *topicName);

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