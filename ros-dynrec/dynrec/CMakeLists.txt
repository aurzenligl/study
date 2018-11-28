cmake_minimum_required(VERSION 2.8.3)
project(dynrec)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
)

generate_dynamic_reconfigure_options(
   cfg/Lorem.cfg
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynrec_node src/node.cpp)

add_dependencies(dynrec_node dynrec_gencfg)

target_link_libraries(dynrec_node
  ${catkin_LIBRARIES}
)
