cmake_minimum_required(VERSION 2.8.3)
project(d)

set(PACKAGE_DEPENDENCIES
  a
  b
  c
)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)
