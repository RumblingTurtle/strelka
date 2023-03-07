FILE(GLOB_RECURSE OSQP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.c)

add_library(osqp SHARED ${OSQP_SOURCES})
target_include_directories(osqp PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}/include/
${CMAKE_CURRENT_SOURCE_DIR}/include/osqp
${CMAKE_CURRENT_SOURCE_DIR}/include/osqp/linux
${CMAKE_CURRENT_SOURCE_DIR}/lin_sys/direct/qdldl
${CMAKE_CURRENT_SOURCE_DIR}/lin_sys/direct/qdldl/qdldl_sources/include
${CMAKE_CURRENT_SOURCE_DIR}/lin_sys/direct/qdldl/amd/include
)
target_link_libraries(osqp dl)