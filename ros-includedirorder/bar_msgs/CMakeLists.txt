cmake_minimum_required(VERSION 2.8.3)
project(bar_msgs)

find_package(catkin REQUIRED message_generation)

add_message_files(
  DIRECTORY msg
  FILES
  Bar.msg
)

generate_messages(
  DEPENDENCIES
  LANGS gencpp genpy
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
