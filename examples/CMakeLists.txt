print_pkg_header(examples)

SET(DEPENDENCY_LIBRARIES 
    strelka
)

FILE(GLOB_RECURSE SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

foreach(file_path ${SRC_FILES})
    string( REPLACE ".cpp" "" new_name ${file_path} )
    get_filename_component(filename ${new_name} NAME)
    message("\t${file_path}")
    add_executable( ${filename} ${file_path} )
    target_link_libraries(${filename} ${DEPENDENCY_LIBRARIES})
endforeach()