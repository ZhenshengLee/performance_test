cmake_minimum_required(VERSION 2.8.3)
project(performance_test_ros1_publisher)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp performance_test_ros1_msgs)

include(ExternalProject)

set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)
externalproject_add(
  tclap
  GIT_REPOSITORY http://git.code.sf.net/p/tclap/code
  GIT_TAG 1.4.0-rc1
  GIT_SHALLOW TRUE
  PREFIX ${EXTERNAL_INSTALL_LOCATION}
  SOURCE_DIR ${EXTERNAL_INSTALL_LOCATION}/tclap
  CMAKE_COMMAND ""
  CONFIGURE_COMMAND ""
  UPDATE_COMMAND ""
  INSTALL_COMMAND ""
  BUILD_COMMAND ""
)
include_directories(SYSTEM ${EXTERNAL_INSTALL_LOCATION}/tclap/include)

## Declare a catkin package
catkin_package()

## Build the publisher
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(publisher src/publisher.cpp)
add_dependencies(publisher tclap)
target_link_libraries(publisher
  ${catkin_LIBRARIES}
)

add_executable(check_master src/check_master.cpp)
target_link_libraries(check_master
  ${catkin_LIBRARIES}
)

install(TARGETS publisher check_master
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
