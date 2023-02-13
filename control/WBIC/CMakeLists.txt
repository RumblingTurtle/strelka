PrintPackageHeader(control/WBIC)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third-party/Goldfarb_Optimizer)

SET(STRELKA_CONTROL_WBIC_DEPENDENCY_HEADERS 
    ${Eigen_INCLUDE_DIRS};
    ${STRELKA_COMMON_HEADERS};
    ${CMAKE_CURRENT_SOURCE_DIR}/third-party;
)

FILE(GLOB_RECURSE STRELKA_CONTROL_WBIC_SOURCES src/*.cpp)

set(STRELKA_CONTROL_WBIC_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/WBIC
)

add_library(strelka_control_wbic ${STRELKA_CONTROL_WBIC_SOURCES})
target_include_directories(strelka_control_wbic PUBLIC ${STRELKA_CONTROL_WBIC_HEADERS} ${STRELKA_CONTROL_WBIC_DEPENDENCY_HEADERS})
target_link_libraries(strelka_control_wbic Goldfarb_Optimizer)


