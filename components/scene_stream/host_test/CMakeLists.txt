cmake_minimum_required(VERSION 3.14)
project(scene_stream_host_test C CXX)

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

# scene_stream depends on laser_command, which depends on byte_queue; pull all
# three C sources into the test build.
add_executable(test_scene_stream
    test_scene_stream.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../scene_stream.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../laser_command/laser_command.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../byte_queue/byte_queue.c)
target_include_directories(test_scene_stream PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../laser_command/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../byte_queue/include)
target_link_libraries(test_scene_stream PRIVATE gtest_main)

gtest_discover_tests(test_scene_stream)
