#ifndef TYPEDEFS_H
#define TYPEDEFS_H
#include <chrono>
#include <iostream>
typedef std::chrono::time_point<std::chrono::high_resolution_clock>
    chrono_time_point;

#define print(x) std::cout << x << std::endl

#endif // TYPEDEFS_H