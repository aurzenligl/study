cmake_minimum_required(VERSION 3.0)
project(cppapp)

find_package(Boost COMPONENTS program_options)

set(default_build_type Debug)
set(allowed_build_types Debug Release)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
        STRING "Choose the type of build." FORCE)
endif()
list (FIND allowed_build_types ${CMAKE_BUILD_TYPE} _index)
if (${_index} EQUAL -1)
    message(FATAL_ERROR "Wrong build type ${CMAKE_BUILD_TYPE}, expected: ${allowed_build_types}")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(GENERIC_FLAGS "-std=c++11 -Wall -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${GENERIC_FLAGS} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${GENERIC_FLAGS} -O3")

add_subdirectory(src)
