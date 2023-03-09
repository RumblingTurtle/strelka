
macro(print_pkg_header name)
message("_________________________________________\n
\t\t\t\t ${name} package
_________________________________________")
endmacro()

macro(print_pkg_footer)
message("------------------------------------------\n")
endmacro()

macro(define_package)
    set( _OPTIONS_ARGS )
    set( _ONE_VALUE_ARGS LIBRARY HEADERS)
    set( _MULTI_VALUE_ARGS  DEPENDS )

    cmake_parse_arguments( _define_package "${_OPTIONS_ARGS}" "${_ONE_VALUE_ARGS}" "${_MULTI_VALUE_ARGS}" ${ARGN} )

    if( _define_package_HEADERS )
        string(TOLOWER ${_define_package_HEADERS} PACKAGE_NAME_LOWER)
    else()
        if( _define_package_LIBRARY )
            string(TOLOWER ${_define_package_LIBRARY} PACKAGE_NAME_LOWER)
        else()
            message( FATAL_ERROR "define_package: 'HEADERS' or 'LIBRARY' argument required." )
        endif()
    endif()

    print_pkg_header(${PACKAGE_NAME_LOWER})

    if( _define_package_DEPENDS )
        message("Depends on: ")
        foreach(DEPENDENCY_NAME ${_define_package_DEPENDS})
            string(TOUPPER ${DEPENDENCY_NAME} DEPENDENCY_NAME_UPPER)
            message("\t" ${DEPENDENCY_NAME})
        endforeach()
    endif()


    if( _define_package_LIBRARY )
        FILE(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
            
        add_library(${PACKAGE_NAME_LOWER} ${SRC_FILES})
        target_include_directories(${PACKAGE_NAME_LOWER} PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        )
        target_link_libraries(${PACKAGE_NAME_LOWER} ${_define_package_DEPENDS})

        install(
            TARGETS ${PACKAGE_NAME_LOWER}
            LIBRARY DESTINATION bin)
    else()
        add_library(${PACKAGE_NAME_LOWER} INTERFACE)
        target_include_directories(${PACKAGE_NAME_LOWER} INTERFACE ./include)
    endif()

    install(DIRECTORY include/ DESTINATION include)

    print_pkg_footer()
endmacro()