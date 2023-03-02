#ifndef MOVING_WINDOW_FILTER_3D_H
#define MOVING_WINDOW_FILTER_3D_H

#include <common/typedefs.hpp>
#include <deque>

namespace strelka {
namespace filters {
/**
 * @brief  A stable O(1) moving filter for incoming data streams.
 *
 * We implement the Neumaier's algorithm to calculate the moving window
 * average, which is numerically stable.
 */
template <typename T> class MovingWindowFilter3D {

  // The moving window sum.
  T _sum[3];
  // The correction term to compensate numerical precision loss during
  // calculation.
  T _correction[3];
  int _windowSize;

  std::deque<T> deques[3];

  /*
   * Update the moving window sum using Neumaier's algorithm.
   *
   * For more details please refer to:
   * https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements
   */
  static void neumaierSum(T &sum, T &correction, T value) {
    T newSum = sum + value;
    if (std::abs(sum) >= std::abs(value)) {
      // If sum is bigger, (low - order) digits of value are lost.
      correction += (sum - newSum) + value;
    } else {
      // (low - order) digits of sum are lost
      correction += (value - newSum) + sum;
    }
    sum = newSum;
  }

public:
  MovingWindowFilter3D(int windowSize = 50)
      : _correction{0, 0, 0}, _sum{0, 0, 0}, _windowSize(windowSize) {
    assert(windowSize > 0);
  }

  /**
   * @brief Computes the moving window average in O(1) time.
   *
   * @param newValues The new values to enter the moving window.
   */
  Vec3<T> getAverage(Vec3<T> newValues) {
    Vec3<T> average;
    for (int dim = 0; dim < 3; dim++) {
      int queueSize = deques[dim].size();
      if (queueSize == _windowSize) {
        neumaierSum(_sum[dim], _correction[dim], -deques[dim].front());
        deques[dim].pop_front();
      }
      neumaierSum(_sum[dim], _correction[dim], newValues[dim]);
      deques[dim].push_back(newValues[dim]);

      average(dim) = (_sum[dim] + _correction[dim]) / _windowSize;
    }

    return average;
  }
};
} // namespace filters
} // namespace strelka

#endif // MOVING_WINDOW_FILTER_3D_H