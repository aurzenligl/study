cmake_minimum_required(VERSION 2.8.3)
project(foo)

find_package(catkin REQUIRED COMPONENTS
  foo_msgs
  bar_msgs
  baz_msgs
)

catkin_package()

add_library(${PROJECT_NAME}
  src/foo.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
