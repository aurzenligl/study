cmake_minimum_required(VERSION 3.0)
project(cppapp)

find_package(Boost COMPONENTS program_options)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror -g -O0")

add_subdirectory(src)
