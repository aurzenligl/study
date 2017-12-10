cmake_minimum_required(VERSION 3.0)
project(cppapp)

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

set(GENERIC_FLAGS "-std=c++11 -Wall -Werror -ftemplate-depth=1024")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${GENERIC_FLAGS} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${GENERIC_FLAGS} -O3 -DNDEBUG")

# paths

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# dependencies

find_package(Boost COMPONENTS program_options)
find_package(OpenCV REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# sub-scripts

add_subdirectory(src)
