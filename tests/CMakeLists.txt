PrintPackageHeader(tests)

SET(STRELKA_TESTS_DEPENDENCY_HEADERS 
    ${Eigen_INCLUDE_DIRS}
    ${STRELKA_COMMON_HEADERS}
    ${STRELKA_INTERFACES_HEADERS}
    ${STRELKA_STATE_ESTIMATION_HEADERS}
    ${STRELKA_CONTROL_HEADERS}
)

SET(STRELKA_TESTS_DEPENDENCY_LIBRARIES 
    strelka_interfaces
    strelka_state_estimation
    strelka_control_wbic_controller
)

file(GLOB STRELKA_TESTS_SOURCES LIST_DIRECTORIES true  ${CMAKE_CURRENT_SOURCE_DIR}/src/*/*.cpp)

foreach(file_path ${STRELKA_TESTS_SOURCES})
    string( REPLACE ".cpp" "" new_name ${file_path} )
    get_filename_component(filename ${new_name} NAME)
    message("Building " ${filename})
    add_executable( ${filename} ${file_path} )
    #target_include_directories(${filename} ${STRELKA_TESTS_DEPENDENCY_HEADERS})
    target_link_libraries(${filename} ${STRELKA_TESTS_DEPENDENCY_LIBRARIES})
endforeach()
