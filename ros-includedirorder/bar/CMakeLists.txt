cmake_minimum_required(VERSION 2.8.3)
project(bar)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED foo)

catkin_package()

add_executable(${PROJECT_NAME}
  src/bar.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
