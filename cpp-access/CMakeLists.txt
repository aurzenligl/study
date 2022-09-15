cmake_minimum_required(VERSION 3.0.0)
project(cpp-access VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(
  -Wall
  -Werror
  -Wpedantic
)

add_executable(cpp-access cpp-access.cpp)
