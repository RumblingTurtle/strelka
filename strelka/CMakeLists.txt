print_pkg_header(strelka)

FILE(GLOB_RECURSE SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_library(strelka SHARED ${SRC_FILES})

if(SRC_FILES)
    foreach(file_path ${SRC_FILES})
        message("\t${file_path}")
    endforeach()
endif()

target_include_directories(
    strelka PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(strelka osqp wbic strelka_messages)

install(TARGETS strelka osqp wbic goldfarb
    LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

print_pkg_footer()