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
  Averaging.action
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

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)
link_libraries(${catkin_LIBRARIES})

add_executable(server src/server.cpp)
add_dependencies(server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(client src/client.cpp)
add_dependencies(client ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(fibonacci_server src/fibonacci_server.cpp)
add_dependencies(fibonacci_server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(fibonacci_client src/fibonacci_client.cpp)
add_dependencies(fibonacci_client ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(fibonacci_client_cbs src/fibonacci_client_cbs.cpp)
add_dependencies(fibonacci_client_cbs ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(averaging_server src/averaging_server.cpp)
add_dependencies(averaging_server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(averaging_client src/averaging_client.cpp)
add_dependencies(averaging_client ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
