FILE(GLOB_RECURSE WBIC_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(wbic OBJECT ${WBIC_SOURCES})
target_include_directories(wbic PUBLIC include/)
set_property(TARGET wbic PROPERTY POSITION_INDEPENDENT_CODE ON)
