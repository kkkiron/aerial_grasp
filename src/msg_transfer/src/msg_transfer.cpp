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
                                     &msg_transfer::msg_transferNode::optitrack_msgCallBack,
                                     this);
}

void msg_transfer::msg_transferNode::optitrack_msgCallBack(const opti_msgs::Odom::ConstPtr& opti_msg){
    if(!received_desired_pose_)
    {
        return;
    }

    if(opti_msg->rigidBodyID == this->id_)
    {
        odom_msg_.pose.pose.position.x = opti_msg->position.x;
        odom_msg_.pose.pose.position.y = opti_msg->position.y;
        odom_msg_.pose.pose.position.z = opti_msg->position.z;
    }
    
    if(opti_msg->rigidBodyID == this->target_rigidbody_id_ && is_track_)
    {
        cmd_msg_.position.x = x_act_;
        cmd_msg_.position.y = y_act_;
        cmd_msg_.position.z = z_act_;
    }
}

void msg_transfer::msg_transferNode::consoleCallBack(const arm_test::position::ConstPtr& rel_msg){
    x_rel_ = rel_msg->x_relative;
    y_rel_ = rel_msg->y_relative;
    z_rel_ = rel_msg->z_relative;
    x_act_ = x_ref_ + x_rel_;
    y_act_ = y_ref_ + y_rel_;
    z_act_ = z_ref_ + z_rel_;
}

void msg_transfer::msg_transferNode::is_trackCallBack(const arm_test::track::ConstPtr& is_track){
    is_track_ = is_track->is_track;

    if(is_track_) {
        ROS_INFO_STREAM("track to target");
    }
    else{
        ROS_INFO_STREAM("untrack to target");
    }
}