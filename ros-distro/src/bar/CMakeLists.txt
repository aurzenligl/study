cmake_minimum_required(VERSION 2.8.3)
project(bar)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS foo)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_main src/bar.cpp)
set_target_properties(${PROJECT_NAME}_main PROPERTIES OUTPUT_NAME bar PREFIX "")
target_link_libraries(${PROJECT_NAME}_main
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
