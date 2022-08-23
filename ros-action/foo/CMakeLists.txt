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
  Fibonacci.action
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package()

set(CMAKE_CXX_STANDARD 17)

add_compile_options(
  -Wall
  -Werror
  -Wpedantic
)

add_executable(server src/server.cpp)
target_include_directories(server SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(server ${catkin_LIBRARIES})
add_dependencies(server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(client src/client.cpp)
target_include_directories(client SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(client ${catkin_LIBRARIES})
add_dependencies(client ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(fibonacci_server src/fibonacci_server.cpp)
target_include_directories(fibonacci_server SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(fibonacci_server ${catkin_LIBRARIES})
add_dependencies(fibonacci_server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(fibonacci_client src/fibonacci_client.cpp)
target_include_directories(fibonacci_client SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(fibonacci_client ${catkin_LIBRARIES})
add_dependencies(fibonacci_client ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
