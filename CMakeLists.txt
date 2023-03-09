cmake_minimum_required(VERSION 3.0.2)
project(strelka LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(Macros)

find_package(lcm REQUIRED)
find_package(Eigen3 REQUIRED)

if (LCM_FOUND)
    include_directories(${LCM_INCLUDE_DIRS})
else()
    message( FATAL_ERROR "LCM was not found. https://github.com/lcm-proj/lcm" )
endif (LCM_FOUND)

# Used for OSQP
add_definitions(-DDFLOAT) 

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O3  -march=native -std=gnu99 -I.")

add_subdirectory(third_party)
add_subdirectory(strelka_common)
add_subdirectory(strelka_messages)
add_subdirectory(strelka_filters)
add_subdirectory(strelka_control)
add_subdirectory(strelka_state_estimation)
add_subdirectory(strelka_robots)
add_subdirectory(examples)
add_subdirectory(tests)