#include <strelka/filters/FirstOrderLPF.hpp>

namespace strelka {
namespace filters {

template <typename T>
FirstOrderLPF<T>::FirstOrderLPF(T samplePeriod, T cutoffFrequency)
    : firstValueRecieved(false), cutoffFrequency(cutoffFrequency) {
  assert(cutoffFrequency > 0 && samplePeriod > 0);

  alpha = (2 - samplePeriod * cutoffFrequency) /
          (2 + samplePeriod * cutoffFrequency);
  beta = samplePeriod * cutoffFrequency / (2 + samplePeriod * cutoffFrequency);
}

template <typename T> T FirstOrderLPF<T>::filter(T x) {
  if (!firstValueRecieved) {
    firstValueRecieved = true;
    xPrev = x;
    yPrev = x;
    return x;
  }

  float y = alpha * yPrev + beta * (x + xPrev);

  xPrev = x;
  yPrev = y;
  return y;
}

template class FirstOrderLPF<float>;
} // namespace filters
} // namespace strelka