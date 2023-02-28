/**
 * @file utilities.hpp
 * Utility functions
 *
 */
#ifndef STRELKA_UTILITIES_H
#define STRELKA_UTILITIES_H
#include <chrono>
#include <iostream>

/**
 * @brief Returns time of execution of a given callable object/lambda in
 * milliseconds
 *
 * @param functor Callable to run
 * @return float runtime of functor in milliseconds
 */
template <typename Callable> float executionTime(Callable functor) {

  std::chrono::time_point<std::chrono::high_resolution_clock> t =
      std::chrono::high_resolution_clock::now();
  functor();
  float ms = std::chrono::duration_cast<std::chrono::microseconds>(
                 std::chrono::high_resolution_clock::now() - t)
                 .count() /
             1000.0f;
  return ms;
}

#endif // STRELKA_UTILITIES_H