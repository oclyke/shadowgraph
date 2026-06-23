cmake_minimum_required(VERSION 3.14)
project(point_ring_host_test C CXX)

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

add_executable(test_point_ring
    test_point_ring.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../point_ring.c)
target_include_directories(test_point_ring PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_link_libraries(test_point_ring PRIVATE gtest_main Threads::Threads)

gtest_discover_tests(test_point_ring)
