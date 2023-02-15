PrintPackageHeader(state_estimation)

SET(STRELKA_STATE_ESTIMATION_DEPENDENCY_HEADERS 
    ${Eigen_INCLUDE_DIRS};
    ${STRELKA_MESSAGES_HEADERS};
    ${STRELKA_COMMON_HEADERS};
    ${STRELKA_ROBOTS_HEADERS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include;
)

SET(STRELKA_STATE_ESTIMATION_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KalmanFilterObserver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SlowdownEstimator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/A1/A1StateEstimator.cpp
)

set(STRELKA_STATE_ESTIMATION_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(strelka_state_estimation ${STRELKA_STATE_ESTIMATION_SOURCES})
target_include_directories(strelka_state_estimation PUBLIC ${STRELKA_STATE_ESTIMATION_DEPENDENCY_HEADERS})
target_link_libraries(strelka_state_estimation PUBLIC strelka_robots)
