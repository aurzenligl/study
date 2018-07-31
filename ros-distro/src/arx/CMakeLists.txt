cmake_minimum_required(VERSION 2.8.3)
project(arx)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS bar)

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

add_executable(arx src/arx.cpp)
target_link_libraries(arx
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS arx
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
