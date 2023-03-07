cmake_minimum_required(VERSION 2.8.3)
project(strelka LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)

message("Build type ${CMAKE_BUILD_TYPE}")

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

# Setup gtest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)


add_subdirectory(third_party)
add_subdirectory(common)
add_subdirectory(messages)

include_directories(${STRELKA_MESSAGES_INCLUDE_DIRS})
include_directories(${STRELKA_COMMON_INCLUDE_DIRS})

add_subdirectory(filters)
add_subdirectory(control)
add_subdirectory(state_estimation)
add_subdirectory(robots)
add_subdirectory(tests)
add_subdirectory(examples)
