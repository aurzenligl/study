cmake_minimum_required(VERSION 2.8.3)
project(bar)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS foo)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  SKIP_CMAKE_CONFIG_GENERATION
  SKIP_PKG_CONFIG_GENERATION
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bar_main src/bar.cpp)
set_target_properties(bar_main PROPERTIES OUTPUT_NAME bar)
target_link_libraries(bar_main
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS bar_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION bin
  # TODO: why bin is not the default, what does this break?
)
