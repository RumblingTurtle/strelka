cmake_minimum_required(VERSION 3.0.2)
project(strelka LANGUAGES C CXX)
set(strelka_VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 14)

enable_testing()

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(Macros)
include(GNUInstallDirs)

find_package(lcm REQUIRED)
find_package(Eigen3 REQUIRED)

if (LCM_FOUND)
    include_directories(${LCM_INCLUDE_DIRS})
else()
    message( FATAL_ERROR "LCM was not found. https://github.com/lcm-proj/lcm" )
endif (LCM_FOUND)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG -fPIC -fPIE -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -fPIE")

set(CMAKE_C_FLAGS_RELEASE "-O3  -march=native -std=gnu99 -I. -fPIC -fPIE  -fno-omit-frame-pointer")
set(CMAKE_C_FLAGS_DEBUG "-fPIC -fPIE")

add_subdirectory(third_party)
add_subdirectory(strelka_messages)
add_subdirectory(strelka)
add_subdirectory(examples)
add_subdirectory(tests)