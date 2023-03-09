print_pkg_header(tests)

# Setup gtest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)

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
