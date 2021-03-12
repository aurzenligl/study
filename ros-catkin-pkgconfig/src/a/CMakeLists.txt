cmake_minimum_required(VERSION 2.8.3)
project(a)

set(PACKAGE_DEPENDENCIES)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS Boost OpenCV PCL
)
