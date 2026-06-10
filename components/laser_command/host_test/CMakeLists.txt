cmake_minimum_required(VERSION 3.14)
project(laser_command_host_test C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Vendored googletest (see third-party/github.com/google/googletest).
set(GTEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../third-party/github.com/google/googletest)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(BUILD_GMOCK   OFF CACHE BOOL "" FORCE)
add_subdirectory(${GTEST_DIR} ${CMAKE_BINARY_DIR}/googletest EXCLUDE_FROM_ALL)

enable_testing()
include(GoogleTest)

# laser_command depends on byte_queue; pull both C sources into the test build.
add_executable(test_laser_command
    test_laser_command.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../laser_command.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../byte_queue/byte_queue.c)
target_include_directories(test_laser_command PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../byte_queue/include)
target_link_libraries(test_laser_command PRIVATE gtest_main)

gtest_discover_tests(test_laser_command)
