cmake_minimum_required(VERSION 2.8.3)
project(msg_transfer)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

add_compile_options(-std=c++11)

SET(SRC_LIST src/main.cpp src/msg_transfer.cpp)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS 
        message_runtime
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
)

add_executable(msg_transfer_node
    ${SRC_LIST}
)

target_link_libraries(msg_transfer_nod ${catkin_LIBRARIES})