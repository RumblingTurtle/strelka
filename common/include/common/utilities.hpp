/**
 * @file utilities.hpp
 * Utility functions
 *
 */
#ifndef STRELKA_UTILITIES_H
#define STRELKA_UTILITIES_H
#include <common/typedefs.hpp>
#include <iostream>

/**
 * @brief Returns time difference between to
 * std::chrono::time_point<std::chrono::high_resolution_clock> instances in
 * milliseconds
 */
inline double timePointDiffInMilliseconds(ChronoTimePoint a,
                                          ChronoTimePoint b) {
  return std::chrono::duration_cast<std::chrono::microseconds>(a - b).count() /
         1000.0;
}

/**
 * @brief Returns time difference between to
 * std::chrono::time_point<std::chrono::high_resolution_clock> instances in
 * seconds
 */
inline double timePointDiffInSeconds(ChronoTimePoint a, ChronoTimePoint b) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(a - b)
      .count();
}

/**
 * @brief Get current wall time using high_resolution_clock
 *
 * @return ChronoTimePoint
 */
inline ChronoTimePoint getWallTime() {
  return std::chrono::high_resolution_clock::now();
}

/**
 * @brief Returns time of execution of a given callable object/lambda in
 * milliseconds
 *
 * @param functor Callable to run
 * @return double runtime of functor in milliseconds
 */
template <typename Callable> inline double executionTime(Callable functor) {

  ChronoTimePoint t = getWallTime();
  functor();
  return timePointDiffInMilliseconds(getWallTime(), t);
}

template <typename T> inline T clamp(T v, T low, T high) {
  return v < low ? low : v > high ? high : v;
}
#endif // STRELKA_UTILITIES_H