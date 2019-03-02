cmake_minimum_required(VERSION 3.4)
project(measured)

set (CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED)

file(GLOB_RECURSE srcs "src/*.cpp")

add_library(measured ${srcs})
target_link_libraries(measured PUBLIC
    Boost::boost
)
set_property(TARGET measured PROPERTY RULE_LAUNCH_COMPILE
    ${CMAKE_SOURCE_DIR}/gcc-wrapper.sh
)
