#ifndef FIRST_ORDER_LFP_H
#define FIRST_ORDER_LFP_H

#include <assert.h>
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
  FirstOrderLPF(T samplePeriod, T cutoffFrequency);
  /**
   * @brief Return filtered value given the new sample x
   */
  T filter(T x);
};
} // namespace filters
} // namespace strelka
#endif // FIRST_ORDER_LFP_H