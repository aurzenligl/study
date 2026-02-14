cmake_minimum_required(VERSION 3.0.2)
project(demo)

set(PACKAGE_DEPENDENCIES
  roscpp
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(talker
  src/topics/talker.cpp
)
target_link_libraries(talker
  ${catkin_LIBRARIES}
)

add_executable(listener
  src/topics/listener.cpp
)
target_link_libraries(listener
  ${catkin_LIBRARIES}
)
