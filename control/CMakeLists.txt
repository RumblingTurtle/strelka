PrintPackageHeader(control)

add_subdirectory(WBIC)

SET(STRELKA_CONTROL_DEPENDENCY_HEADERS 
    ${Eigen_INCLUDE_DIRS};
    ${STRELKA_MESSAGES_HEADERS};
    ${STRELKA_COMMON_HEADERS};
    ${CMAKE_CURRENT_SOURCE_DIR}/third-party;
)

set(STRELKA_CONTROL_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/WBIC/include
)

FILE(GLOB_RECURSE STRELKA_CONTROL_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(strelka_control ${STRELKA_CONTROL_SOURCES})
target_include_directories(strelka_control PUBLIC ${STRELKA_CONTROL_HEADERS} ${STRELKA_CONTROL_DEPENDENCY_HEADERS})
target_link_libraries(strelka_control strelka_control_wbic)



