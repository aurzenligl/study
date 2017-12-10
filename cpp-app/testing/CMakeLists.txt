cmake_minimum_required(VERSION 3.0)
project(cppapp)
enable_testing()

# flags

set(default_build_type Debug)
set(allowed_build_types Debug Release)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "" FORCE)
endif()

list (FIND allowed_build_types ${CMAKE_BUILD_TYPE} _index)
if (${_index} EQUAL -1)
    message(FATAL_ERROR "Wrong build type ${CMAKE_BUILD_TYPE}, expected: ${allowed_build_types}")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(GENERIC_FLAGS "-std=c++11 -Wall -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${GENERIC_FLAGS} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${GENERIC_FLAGS} -O3")

# paths

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# dependencies

set(GTEST_ROOT ${CMAKE_SOURCE_DIR}/vendor/gtest-1.8.0)
set(BUILD_GTEST ON)
set(BUILD_GMOCK OFF)
add_subdirectory(${GTEST_ROOT})
set(GTEST_INCLUDE_DIRS ${GTEST_ROOT}/googletest/include)

# sub-scripts

add_subdirectory(src)
add_subdirectory(test)
