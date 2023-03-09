print_pkg_header(examples)

SET(DEPENDENCY_LIBRARIES 
    strelka_robots
    strelka_state_estimation
    strelka_control
    strelka_common
)

FILE(GLOB STRELKA_EXAMPLES_SOURCES LIST_DIRECTORIES true "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

foreach(file_path ${STRELKA_EXAMPLES_SOURCES})
    string( REPLACE ".cpp" "" new_name ${file_path} )
    get_filename_component(filename ${new_name} NAME)
    message("Prepping " ${filename})
    add_executable( ${filename} ${file_path} )
    target_link_libraries(${filename} ${DEPENDENCY_LIBRARIES})
endforeach()