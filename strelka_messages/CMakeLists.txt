message(STATUS "Generating header files strelka_messages package")
execute_process ( COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/generate_msg_headers.sh -a
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ )

define_package(
    HEADERS
        strelka_messages
)