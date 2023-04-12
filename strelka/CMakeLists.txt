print_pkg_header(strelka)

FILE(GLOB_RECURSE SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_library(strelka SHARED ${SRC_FILES} $<TARGET_OBJECTS:wbic> $<TARGET_OBJECTS:osqp>)


if(SRC_FILES)
    foreach(file_path ${SRC_FILES})
        message("\t${file_path}")
    endforeach()
endif()

get_target_property(WBIC_INLCUDE_DIRS wbic INCLUDE_DIRECTORIES)
get_target_property(OSQP_INLCUDE_DIRS osqp INCLUDE_DIRECTORIES)
get_target_property(STRELKA_MESSAGES_INLCUDE_DIRS strelka_messages INTERFACE_INCLUDE_DIRECTORIES)

set(DEPENDENCY_INCLUDE_DIRS 
    ${LCM_INCLUDE_DIRS}
    ${WBIC_INLCUDE_DIRS} 
    ${OSQP_INLCUDE_DIRS} 
)

set(INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${STRELKA_MESSAGES_INLCUDE_DIRS})
    
target_include_directories(strelka 
    PUBLIC   
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_include_directories(strelka PRIVATE ${DEPENDENCY_INCLUDE_DIRS})

target_link_libraries(strelka lcm strelka_messages dl)

#################
# Install targets
#################

install(TARGETS strelka strelka_messages
  EXPORT strelkaTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

#################
# Install target metadata
#################

set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/cmake/strelka)

export(EXPORT strelkaTargets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/strelkaTargets.cmake
  NAMESPACE strelka::)
install(EXPORT strelkaTargets
  FILE strelkaTargets.cmake
  NAMESPACE strelka::
  DESTINATION ${ConfigPackageLocation})

#################
# Generate configs
#################
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/strelkaConfigVersion.cmake
    VERSION ${strelka_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  strelkaConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/strelkaConfig.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR)

# Install configs only when actually installing the package
install(FILES 
  ${CMAKE_CURRENT_BINARY_DIR}/strelkaConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/strelkaConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation})

################
# Uninstall library
################
add_custom_target("uninstall" COMMENT "Uninstall installed files")
add_custom_command(
    TARGET "uninstall"
    POST_BUILD
    COMMENT "Uninstall files with install_manifest.txt"
    COMMAND xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/../install_manifest.txt || echo Nothing in
            install_manifest.txt to be uninstalled!
)
print_pkg_footer()
