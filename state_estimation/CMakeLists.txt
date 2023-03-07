printPackageHeader(state_estimation)

FILE(GLOB_RECURSE STRELKA_STATE_ESTIMATION_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

SET(DEPENDENCY_LIBRARIES
    strelka_filters 
    strelka_common
    strelka_messages)
    
add_library(strelka_state_estimation ${STRELKA_STATE_ESTIMATION_SOURCES})
target_include_directories(strelka_state_estimation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(strelka_state_estimation ${DEPENDENCY_LIBRARIES})
