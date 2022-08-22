cmake_minimum_required(VERSION 2.8.3)
project(foo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
)

add_action_files(FILES
  DoDishes.action
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package()

add_executable(server src/server.cpp)
target_include_directories(server PUBLIC SYSTEM ${catkin_INCLUDE_DIRS})
target_link_libraries(server ${catkin_LIBRARIES})
add_dependencies(server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(client src/client.cpp)
target_include_directories(client PUBLIC SYSTEM ${catkin_INCLUDE_DIRS})
target_link_libraries(client ${catkin_LIBRARIES})
add_dependencies(client ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
