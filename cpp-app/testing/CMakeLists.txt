# TODO: check coverage

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

set(GENERIC_FLAGS "-std=c++11 -Wall -Werror -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${GENERIC_FLAGS} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${GENERIC_FLAGS} -O3 -DNDEBUG")

# cpu count

execute_process(COMMAND nproc
                RESULT_VARIABLE retcode
                OUTPUT_VARIABLE result
                OUTPUT_STRIP_TRAILING_WHITESPACE)
if(${retcode} STREQUAL 0)
    set(CMAKE_CPU_COUNT ${result})
else()
    set(CMAKE_CPU_COUNT 1)
endif()

# paths

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# dependencies

find_package(PkgConfig)

pkg_check_modules(GTEST REQUIRED gtest)
find_package(benchmark)

# sub-scripts

add_subdirectory(src)
add_subdirectory(test EXCLUDE_FROM_ALL)
#add_subdirectory(vendor EXCLUDE_FROM_ALL)
