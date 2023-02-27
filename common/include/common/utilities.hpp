#ifndef STRELKA_UTILITIES_H
#define STRELKA_UTILITIES_H
#include <chrono>
#include <iostream>

/// @brief Returns run time of a callable
/// @tparam Callable Callable object type
/// @param functor Functor object to run
template <typename Callable> float executionTime(Callable functor) {

  std::chrono::time_point<std::chrono::high_resolution_clock> t =
      std::chrono::high_resolution_clock::now();
  functor();
  float ms = std::chrono::duration_cast<std::chrono::microseconds>(
                 std::chrono::high_resolution_clock::now() - t)
                 .count() /
             1000.0f;
  std::cout << "Time to execute: " << ms << "ms" << std::endl;
}

#endif // STRELKA_UTILITIES_H