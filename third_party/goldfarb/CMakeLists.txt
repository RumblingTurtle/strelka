FILE(GLOB_RECURSE GOLDFARB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

add_library (goldfarb ${GOLDFARB_SOURCES})
target_include_directories(goldfarb PUBLIC include/)