#ifndef MOVING_WINDOW_FILTER_3D_H
#define MOVING_WINDOW_FILTER_3D_H

#include <assert.h>
#include <deque>
#include <strelka_common/typedefs.hpp>

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
  static void neumaierSum(T &sum, T &correction, T value);

public:
  MovingWindowFilter3D(int windowSize = 50);

  /**
   * @brief Computes the moving window average in O(1) time.
   *
   * @param newValues The new values to enter the moving window.
   */
  Vec3<T> getAverage(Vec3<T> newValues);
};
} // namespace filters
} // namespace strelka

#endif // MOVING_WINDOW_FILTER_3D_H