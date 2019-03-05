cmake_minimum_required(VERSION 3.0)
project(measured)
set(CMAKE_CXX_STANDARD 14)

option(STATS "Measure compilation time" OFF)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

file(GLOB_RECURSE srcs "src/*.cpp")
add_library(measured STATIC ${srcs})
target_include_directories(measured PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(measured PUBLIC
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
)
if (STATS)
    set_property(TARGET measured PROPERTY RULE_LAUNCH_COMPILE
        ${CMAKE_SOURCE_DIR}/gcc-wrapper.sh
    )
endif()
