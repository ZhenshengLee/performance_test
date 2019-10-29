cmake_minimum_required(VERSION 2.8.3)
project(performance_test_ros1_publisher)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp performance_test_ros1_msgs)
find_package(Boost COMPONENTS program_options REQUIRED)

## Declare a catkin package
catkin_package()

## Build the publisher
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
