cmake_minimum_required(VERSION 2.8.3)
project(strelka)

set(CMAKE_CXX_FLAGS "-O3 -ggdb")
set(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(lcm REQUIRED)
find_package(Eigen3 REQUIRED)

if (LCM_FOUND)
    include_directories(${LCM_INCLUDE_DIRS})
endif (LCM_FOUND)

macro(PrintPackageHeader name)
message("_________________________________________\n
\t\t\t\t Strelka ${name} package
_________________________________________")
endmacro()

add_subdirectory(common)
add_subdirectory(messages)
add_subdirectory(robots)
add_subdirectory(interfaces)
add_subdirectory(state_estimation)
add_subdirectory(control)
add_subdirectory(tests)
