cmake_minimum_required(VERSION 2.8.3)
project(j)

set(PACKAGE_DEPENDENCIES
  a
  b
  c
  d
  e
  f
  g
  h
  i
)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)
