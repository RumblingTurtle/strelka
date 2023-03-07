printPackageHeader(tests)

SET(DEPENDENCY_LIBRARIES 
    strelka_robots
    strelka_state_estimation
    strelka_control
)

FILE(GLOB STRELKA_TESTS_SOURCES LIST_DIRECTORIES true ${CMAKE_CURRENT_SOURCE_DIR}/src/*/*_test.cpp)

foreach(file_path ${STRELKA_TESTS_SOURCES})
    string( REPLACE ".cpp" "" new_name ${file_path} )
    get_filename_component(filename ${new_name} NAME)
    message("Prepping " ${filename})
    add_executable( ${filename} ${file_path} )
    target_link_libraries(${filename} ${DEPENDENCY_LIBRARIES} GTest::gtest_main)
    gtest_discover_tests(${filename})
endforeach()
