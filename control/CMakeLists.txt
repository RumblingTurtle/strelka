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

add_library(strelka_control_wbic_controller ${CMAKE_CURRENT_SOURCE_DIR}/src/WholeBodyImpedanceController.cpp)
target_include_directories(strelka_control_wbic_controller PUBLIC ${STRELKA_CONTROL_HEADERS} ${STRELKA_CONTROL_DEPENDENCY_HEADERS})
target_link_libraries(strelka_control_wbic_controller strelka_control_wbic)



