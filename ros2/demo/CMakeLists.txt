cmake_minimum_required(VERSION 3.20)
project(demo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

function(custom_executable target)
  cmake_parse_arguments(ARG "" "" "SOURCES;DEPENDENCIES" ${ARGN})
  add_executable(${target} ${ARG_SOURCES})
  target_include_directories(${target} PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )
  target_link_libraries(${target} PRIVATE ${ARG_DEPENDENCIES})
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(talker SOURCES
  src/topics/talker.cpp
  DEPENDENCIES rclcpp::rclcpp ${std_msgs_TARGETS})

custom_executable(listener SOURCES
  src/topics/listener.cpp
  DEPENDENCIES rclcpp::rclcpp ${std_msgs_TARGETS})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
