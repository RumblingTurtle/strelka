FILE(GLOB_RECURSE WBIC_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(wbic ${WBIC_SOURCES})
target_include_directories(wbic PUBLIC include/)
target_link_libraries(wbic goldfarb)