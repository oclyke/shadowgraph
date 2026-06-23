cmake_minimum_required(VERSION 3.14)
project(artnet_control_host_test C CXX)

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
find_package(Threads REQUIRED)

add_executable(test_artnet_control
    test_artnet_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../artnet_control.c)
target_include_directories(test_artnet_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_link_libraries(test_artnet_control PRIVATE gtest_main Threads::Threads)

gtest_discover_tests(test_artnet_control)
