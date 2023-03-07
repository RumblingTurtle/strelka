FILE(GLOB_RECURSE WBIC_SOURCES src/*.cpp)

add_library(wbic SHARED ${WBIC_SOURCES})
target_include_directories(wbic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include/WBIC)
target_link_libraries(wbic Goldfarb_Optimizer)