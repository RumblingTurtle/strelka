printPackageHeader(messages)

message("Generating header files for LCM messages")
execute_process ( COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/generate_msg_headers.sh -a
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ )

add_library(strelka_messages INTERFACE)
target_include_directories(strelka_messages INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/autosrc/include)