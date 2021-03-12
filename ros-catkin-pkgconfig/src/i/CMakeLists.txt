cmake_minimum_required(VERSION 2.8.3)
project(i)

set(PACKAGE_DEPENDENCIES
  a
  b
  c
  d
  e
  f
  g
  h
)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)
