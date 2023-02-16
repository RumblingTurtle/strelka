printPackageHeader(tests)

SET(STRELKA_TESTS_DEPENDENCY_INCLUDE_DIRS 
    ${Eigen_INCLUDE_DIRS}
    ${STRELKA_COMMON_INCLUDE_DIRS}
    ${STRELKA_INTERFACES_INCLUDE_DIRS}
    ${STRELKA_STATE_ESTIMATION_INCLUDE_DIRS}
    ${STRELKA_CONTROL_INCLUDE_DIRS}
)

SET(STRELKA_TESTS_DEPENDENCY_LIBRARIES 
    strelka_robots
    strelka_interfaces
    strelka_state_estimation
    strelka_control
)

file(GLOB STRELKA_TESTS_SOURCES LIST_DIRECTORIES true  ${CMAKE_CURRENT_SOURCE_DIR}/src/*/*.cpp)

foreach(file_path ${STRELKA_TESTS_SOURCES})
    string( REPLACE ".cpp" "" new_name ${file_path} )
    get_filename_component(filename ${new_name} NAME)
    message("Building " ${filename})
    add_executable( ${filename} ${file_path} )
    #target_include_directories(${filename} ${STRELKA_TESTS_DEPENDENCY_INCLUDE_DIRS})
    target_link_libraries(${filename} ${STRELKA_TESTS_DEPENDENCY_LIBRARIES})
endforeach()
