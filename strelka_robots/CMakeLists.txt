print_pkg_header(strelka_robots)

FILE(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

if(SRC_FILES)
    foreach(file_path ${SRC_FILES})
        message("\t${file_path}")
    endforeach()
endif()

add_library(strelka_robots ${SRC_FILES})

target_include_directories(
    strelka_robots PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(strelka_robots PUBLIC strelka lcm)

install(TARGETS strelka_robots
    LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

print_pkg_footer()