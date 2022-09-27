cmake_minimum_required(VERSION 2.8.3)
project(foo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
)

catkin_package()

set(CMAKE_CXX_STANDARD 17)

add_compile_options(
  -Wall
  -Werror
  -Wpedantic
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)
link_libraries(${catkin_LIBRARIES})

add_executable(server
  src/blocking_timer.cpp
  src/server.cpp
)
add_dependencies(server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(race
  src/race.cpp
)
add_dependencies(race ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
