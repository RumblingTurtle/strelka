cmake_minimum_required(VERSION 2.8.3)
project(strelka)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

SET(STRELKA_MESSAGES_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/messages/autosrc/include)
SET(STRELKA_COMMON_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/common/include)

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

add_subdirectory(messages)
add_subdirectory(interfaces)
add_subdirectory(state_estimation)
add_subdirectory(tests)
