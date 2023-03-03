#ifndef FIRST_ORDER_LFP_H
#define FIRST_ORDER_LFP_H
namespace strelka {
namespace filters {

/**
 * @brief Simple discrete time Low Pass Filter
 *
 */
template <typename T> class FirstOrderLPF {
  T xPrev;
  T yPrev;

  T cutoffFrequency;
  T alpha;
  T beta;

  bool firstValueRecieved;

public:
  FirstOrderLPF(T samplePeriod, T cutoffFrequency)
      : firstValueRecieved(false), cutoffFrequency(cutoffFrequency) {
    assert(cutoffFrequency > 0 && samplePeriod > 0);

    alpha = (2 - samplePeriod * cutoffFrequency) /
            (2 + samplePeriod * cutoffFrequency);
    beta =
        samplePeriod * cutoffFrequency / (2 + samplePeriod * cutoffFrequency);
  }

  /**
   * @brief Return filtered value given the new sample x
   */
  T filter(T x) {
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
};
} // namespace filters
} // namespace strelka
#endif // FIRST_ORDER_LFP_H