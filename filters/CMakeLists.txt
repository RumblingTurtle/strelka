PrintPackageHeader(filters)

SET(STRELKA_FILTERS_DEPENDENCY_LIBRARIES strelka_common)

FILE(GLOB_RECURSE STRELKA_FILTERS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(strelka_filters ${STRELKA_FILTERS_SOURCES})
target_include_directories(strelka_filters PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(strelka_filters PUBLIC ${STRELKA_FILTERS_DEPENDENCY_LIBRARIES})
