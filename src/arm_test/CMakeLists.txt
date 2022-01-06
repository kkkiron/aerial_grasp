cmake_minimum_required(VERSION 2.8.3)
project(arm_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
)

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "\n\n OpenCV2 is missing")
endif()

add_compile_options(-std=c++11)

SET(SRC_LIST src/main.cpp src/armtest.cpp)

add_message_files(
    FILES
    gripper.msg
    controls.msg
    point.msg
    position.msg
    track.msg
)

generate_messages(
    DEPENDENCIES 
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS 
        message_runtime
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(armtest_node
    ${SRC_LIST}
)

#add_dependencies(Armtest_node Armtest_node_generate_messages_cpp)

target_link_libraries(armtest_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
