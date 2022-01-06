cmake_minimum_required(VERSION 2.8.3)
project(bebop_odom)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  nav_msgs
  opti_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bebop_joy
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(receiveOdomTest src/receiveOdomTest.cpp)
target_link_libraries(receiveOdomTest ${catkin_LIBRARIES})












