cmake_minimum_required(VERSION 2.8.3)
project(strelka)

#set(CMAKE_CXX_FLAGS "-O3 ")
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-O3 -ggdb")

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(lcm REQUIRED)
find_package(Eigen3 REQUIRED)

if (LCM_FOUND)
    include_directories(${LCM_INCLUDE_DIRS})
endif (LCM_FOUND)

include(Macros)

add_subdirectory(third_party)
add_subdirectory(messages)
add_subdirectory(common)
add_subdirectory(robots)
add_subdirectory(interfaces)
add_subdirectory(state_estimation)
add_subdirectory(control)
add_subdirectory(tests)
