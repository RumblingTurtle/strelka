#include <filters/MovingWindowFilter3D.hpp>

namespace strelka {
namespace filters {

template <typename T>
void MovingWindowFilter3D<T>::neumaierSum(T &sum, T &correction, T value) {
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

template <typename T>
MovingWindowFilter3D<T>::MovingWindowFilter3D(int windowSize)
    : _correction{0, 0, 0}, _sum{0, 0, 0}, _windowSize(windowSize) {
  assert(windowSize > 0);
}

template <typename T>
Vec3<T> MovingWindowFilter3D<T>::getAverage(Vec3<T> newValues) {
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

template class MovingWindowFilter3D<float>;
} // namespace filters
} // namespace strelka
