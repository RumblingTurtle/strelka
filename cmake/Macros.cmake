
macro(printPackageHeader name)
message("_________________________________________\n
\t\t\t\t Strelka ${name} package
_________________________________________")
endmacro()

macro(printPackageFooter name)
message("------------------------------------------\n")
endmacro()

macro(definePackage)
    set( _OPTIONS_ARGS )
    set( _ONE_VALUE_ARGS NAME)
    set( _MULTI_VALUE_ARGS  DEPENDS )

    cmake_parse_arguments( _DEFINEPACKAGE "${_OPTIONS_ARGS}" "${_ONE_VALUE_ARGS}" "${_MULTI_VALUE_ARGS}" ${ARGN} )

    if( _DEFINEPACKAGE_NAME )
    else()
        message( FATAL_ERROR "definePackage: 'NAME' argument required." )
    endif()

    string(TOLOWER ${_DEFINEPACKAGE_NAME} PACKAGE_NAME_LOWER)
    string(TOUPPER ${_DEFINEPACKAGE_NAME} PACKAGE_NAME_UPPER)

    printPackageHeader(${PACKAGE_NAME_LOWER})

    #set($ENV{"STRELKA_${PACKAGE_NAME_UPPER}_HEADERS"} "${CMAKE_CURRENT_SOURCE_DIR}/include")

    message($ENV{STRELKA_${PACKAGE_NAME_UPPER}_HEADERS})
    message(${STRELKA_${PACKAGE_NAME_UPPER}_HEADERS})

    FILE(GLOB_RECURSE PACKAGE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

    if(PACKAGE_SOURCES)
        add_library(strelka_${PACKAGE_NAME_LOWER} ${STRELKA_${PACKAGE_NAME_UPPER}_SOURCES})

        if( _DEFINEPACKAGE_DEPENDS )
            message("Depends on: ")
            foreach(DEPENDENCY_NAME ${_DEFINEPACKAGE_DEPENDS})
                #set ${STRELKA_${PACKAGE_NAME_UPPER}_DEPENDENCY_HEADERS}
                string(TOUPPER ${DEPENDENCY_NAME} DEPENDENCY_NAME_UPPER)
                message("\t" ${DEPENDENCY_NAME})
            endforeach()
        endif()

        target_include_directories(strelka_${PACKAGE_NAME_LOWER} PUBLIC ${PACKAGE_HEADERS_PATH}) #${STRELKA_${PACKAGE_NAME_UPPER}_DEPENDENCY_HEADERS} )
        target_link_libraries(strelka_${PACKAGE_NAME_LOWER} ${_DEFINEPACKAGE_DEPENDS})


    endif()
        
    message("Headers path: ")

    message("Sources: ")
    message(${PACKAGE_SOURCES})
    
    printPackageFooter(NAME)
endmacro()