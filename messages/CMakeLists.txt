printPackageHeader(messages)

message("Generating header files for LCM messages")
execute_process ( COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/generate_msg_headers.sh -a
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ )
                  

SET(STRELKA_MESSAGES_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/autosrc/include PARENT_SCOPE)