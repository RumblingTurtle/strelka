FILE(GLOB_RECURSE GOLDFARB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

add_library (Goldfarb_Optimizer STATIC  ${GOLDFARB_SOURCES})
target_include_directories(Goldfarb_Optimizer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
