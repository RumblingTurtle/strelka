FILE(GLOB_RECURSE GOLDFARB_SOURCES *.cc)

set(GOLDFARB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library (Goldfarb_Optimizer SHARED  ${GOLDFARB_SOURCES})
target_include_directories(Goldfarb_Optimizer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
