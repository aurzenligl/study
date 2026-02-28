cmake_minimum_required(VERSION 3.20)
project(demo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(demo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

macro(custom_library name plugin)
  cmake_parse_arguments(ARG "" "" "SOURCES;DEPENDENCIES" ${ARGN})
  set(library ${name}_component)
  add_library(${library} SHARED ${ARG_SOURCES})
  target_link_libraries(${library} PRIVATE ${ARG_DEPENDENCIES})
  rclcpp_components_register_node(${library} PLUGIN "${plugin}" EXECUTABLE ${name})
  install(TARGETS ${library}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endmacro()

custom_library(talker "demo::Talker" SOURCES
  src/topics/talker.cpp
  DEPENDENCIES rclcpp::rclcpp rclcpp_components::component ${std_msgs_TARGETS} ${demo_msgs_TARGETS})

custom_library(listener "demo::Listener" SOURCES
  src/topics/listener.cpp
  DEPENDENCIES rclcpp::rclcpp rclcpp_components::component ${std_msgs_TARGETS} ${demo_msgs_TARGETS})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
