PrintPackageHeader(robots)

SET(DEPENDENCY_LIBRARIES 
    strelka_control 
    strelka_state_estimation
    lcm
)

FILE(GLOB_RECURSE STRELKA_ROBOTS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(strelka_robots ${STRELKA_ROBOTS_SOURCES})
target_include_directories(strelka_robots PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(strelka_robots PUBLIC ${DEPENDENCY_LIBRARIES})