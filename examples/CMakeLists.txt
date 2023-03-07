printPackageHeader(examples)

SET(STRELKA_EXAMPLES_DEPENDENCY_INCLUDE_DIRS 
    ${Eigen_INCLUDE_DIRS}
    ${STRELKA_COMMON_INCLUDE_DIRS}
    ${STRELKA_INTERFACES_INCLUDE_DIRS}
    ${STRELKA_STATE_ESTIMATION_INCLUDE_DIRS}
    ${STRELKA_CONTROL_INCLUDE_DIRS}
    ${STRELKA_MESSAGES_INCLUDE_DIRS}
    ${STRELKA_ROBOTS_INCLUDE_DIRS}
)

SET(STRELKA_EXAMPLES_DEPENDENCY_LIBRARIES 
    strelka_robots
    strelka_state_estimation
    strelka_control
)

FILE(GLOB STRELKA_EXAMPLES_SOURCES LIST_DIRECTORIES true "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

foreach(file_path ${STRELKA_EXAMPLES_SOURCES})
    string( REPLACE ".cpp" "" new_name ${file_path} )
    get_filename_component(filename ${new_name} NAME)
    message("Prepping " ${filename})
    add_executable( ${filename} ${file_path} )
    target_link_libraries(${filename} ${STRELKA_EXAMPLES_DEPENDENCY_LIBRARIES})
endforeach()
lcm_local_planner