cmake_minimum_required(VERSION 3.15)
project(motor
    VERSION 0.0.1
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

include(GNUInstallDirs)

find_package(serial REQUIRED)

# add_subdirectory(include/motor)

add_library(motor INTERFACE)

target_link_libraries(motor
    INTERFACE
        ${serial_LIBRARIES}
)

target_include_directories(motor
    INTERFACE
        ${serial_INCLUDE_DIRS}
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(TARGETS motor EXPORT ${PROJECT_NAME}-config)

install(
    EXPORT ${PROJECT_NAME}-config # name of .cmake file
    NAMESPACE motor::
    # where the .cmake file will be installed
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../include/${PROJECT_NAME}/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})