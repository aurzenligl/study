cmake_minimum_required(VERSION 3.5.1)
project(t1)

find_package(BehaviorTreeV3 REQUIRED)

add_executable(foo
  src/xdoor.cpp
  src/main.cpp
  src/types.cpp
)
target_link_libraries(foo
  BT::behaviortree_cpp_v3
)
