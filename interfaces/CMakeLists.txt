PrintPackageHeader(interfaces)

SET(STRELKA_INTERFACES_DEPENDENCY_HEADERS 
    ${Eigen_INCLUDE_DIRS};
    ${STRELKA_MESSAGES_HEADERS};
    ${STRELKA_COMMON_HEADERS};
    ${CMAKE_CURRENT_SOURCE_DIR}/include;
)

SET(STRELKA_INTERFACES_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/GazeboInterface.cpp
)

set(STRELKA_INTERFACES_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(strelka_interfaces ${STRELKA_INTERFACES_SOURCES})
target_include_directories(strelka_interfaces PUBLIC ${STRELKA_INTERFACES_DEPENDENCY_HEADERS})
target_link_libraries(strelka_interfaces lcm)

