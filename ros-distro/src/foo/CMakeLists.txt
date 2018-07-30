cmake_minimum_required(VERSION 2.8.3)
project(foo)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES foo
)

###########
## Build ##
###########

include_directories(
  include
)
add_library(foo
  src/foo.cpp
)

#############
## Install ##
#############

install(TARGETS foo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/foo/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
