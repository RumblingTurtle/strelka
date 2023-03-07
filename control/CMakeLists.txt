printPackageHeader(control)

FILE(GLOB_RECURSE STRELKA_CONTROL_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

SET(DEPENDENCY_LIBRARIES 
    strelka_filters 
    strelka_common
    strelka_messages 
    wbic 
    osqp
)

add_library(strelka_control ${STRELKA_CONTROL_SOURCES})
target_include_directories(strelka_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(strelka_control ${DEPENDENCY_LIBRARIES})