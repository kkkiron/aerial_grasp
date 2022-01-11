#include "../include/msg_transfer.h"

msg_transfer::msg_transferNode::msg_transferNode(ros::NodeHandle &n){
    this->InitPublishers(n);
    this->InitSubscribers(n);
}

void msg_transfer::msg_transferNode::InitPublishers(ros::NodeHandle &n){
    OdometryPublisher        = n.advertise<nav_msgs::Odometry>("odom", 10);
    PositionCommandPublisher = n.advertise<quadrotor_msgs::PositionCommand>("cmd", 10);
}

void msg_transfer::msg_transferNode::InitSubscribers(ros::NodeHandle &n){
    OptitrackSubscriber = 
        n.subscribe<opti_msgs::Odom>("/agent/opti_odom",
                                     10,
                                     &msg_transfer::msg_transferNode::GetOptitrackMsgCallBack,
                                     this);
}

void msg_transfer::msg_transferNode::optitrack_msgCallBack(const opti_msgs::Odom::ConstPtr& opti_msg){
    
}