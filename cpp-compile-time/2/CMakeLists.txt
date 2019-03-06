cmake_minimum_required(VERSION 3.0)
project(measured)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(cotire)

option(STATS "Measure compilation time" OFF)
option(COTIRE "Use precompiled headers" OFF)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

file(GLOB_RECURSE srcs "src/*.cpp")
add_library(measured STATIC ${srcs})
target_include_directories(measured PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    src
)
target_link_libraries(measured PUBLIC
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
)
if (STATS)
    include(cotime)
endif()
if (COTIRE)
    set_target_properties(measured PROPERTIES COTIRE_ADD_UNITY_BUILD FALSE)
    cotire(measured)
endif()
