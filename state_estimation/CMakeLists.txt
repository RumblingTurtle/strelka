printPackageHeader(state_estimation)

SET(STRELKA_STATE_ESTIMATION_DEPENDENCY_INCLUDE_DIRS 
    ${Eigen_INCLUDE_DIRS};
    ${STRELKA_MESSAGES_INCLUDE_DIRS};
    ${STRELKA_COMMON_INCLUDE_DIRS};
    ${STRELKA_ROBOTS_INCLUDE_DIRS}
)

SET(STRELKA_STATE_ESTIMATION_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KalmanFilterObserver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SlowdownEstimator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/A1/A1StateEstimator.cpp
)

add_library(strelka_state_estimation ${STRELKA_STATE_ESTIMATION_SOURCES})
target_include_directories(strelka_state_estimation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${STRELKA_STATE_ESTIMATION_DEPENDENCY_INCLUDE_DIRS})
target_link_libraries(strelka_state_estimation PUBLIC strelka_robots)

set(STRELKA_STATE_ESTIMATION_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    PARENT_SCOPE
)