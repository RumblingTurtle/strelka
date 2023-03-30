message(STATUS "Generating header files strelka_messages package")
execute_process ( COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/generate_msg_headers.sh -a
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ )

print_pkg_header(strelka_messages)

add_library(strelka_messages INTERFACE)
target_include_directories(strelka_messages INTERFACE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/
  DESTINATION include)
  
print_pkg_footer()