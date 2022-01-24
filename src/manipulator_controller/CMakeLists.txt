cmake_minimum_required(VERSION 3.0.2)
project(manipulator_controller)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs 
  roscpp 
  rospy 
  std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include/../../../devel/include
  include/../../../devel/include/manipulator_msgs
  /usr/local/include/serial
)

add_executable(manipulator_controller_node src/main.cpp)

target_link_libraries(manipulator_controller_node
  ${catkin_LIBRARIES}
  serial
)