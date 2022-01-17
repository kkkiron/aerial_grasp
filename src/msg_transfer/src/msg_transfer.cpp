#include "../include/msg_transfer.h"

msg_transfer::msg_transferNode::msg_transferNode(ros::NodeHandle &n){
    this->InitPublishers(n);
    this->InitSubscribers(n);

    set_point_server_ = 
        n.advertiseService("/set_point",
                             &msg_transfer::msg_transferNode::setPoint, 
                             this);

    publish_timer_ = 
        n.createTimer(ros::Duration(publish_time_), 
                      &msg_transfer::msg_transferNode::odom_cmdPublisher, 
                      this);
}

void msg_transfer::msg_transferNode::InitPublishers(ros::NodeHandle &n){
    OdometryPublisher_        = n.advertise<nav_msgs::Odometry>("odom", 10);
    PositionCommandPublisher_ = n.advertise<msg_transfer::PositionCommand>("cmd", 10);
}

void msg_transfer::msg_transferNode::InitSubscribers(ros::NodeHandle &n){
    OptitrackSubscriber_ = 
        n.subscribe<opti_msgs::Odom>("/agent/opti_odom",
                                     10,
                                     &msg_transfer::msg_transferNode::optitrack_msgCallBack,
                                     this);
    IstrackSubscriber_ =
        n.subscribe<arm_test::track>("/track",
                                     10,
                                     &msg_transfer::msg_transferNode::is_trackCallBack,
                                     this);
    ConsoleSubscriber_ =
        n.subscribe<arm_test::position>("/position",
                                     10,
                                     &msg_transfer::msg_transferNode::consoleCallBack,
                                     this);
    OdomSubscriber_ =
        n.subscribe<nav_msgs::Odometry>("odom",
                                     10,
                                     &msg_transfer::msg_transferNode::odomCallBack,
                                     this);
    
}

void msg_transfer::msg_transferNode::odomCallBack(const nav_msgs::Odometry::ConstPtr& opti_msg){
    std::cout << "odom callback" << std::endl;
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
        x_ref_ = opti_msg->position.x;
        y_ref_ = opti_msg->position.y;
        z_ref_ = opti_msg->position.z;
        x_act_ = x_ref_ + x_rel_ + x_bias_;
        y_act_ = y_ref_ + y_rel_ + y_bias_;
        z_act_ = z_ref_ + z_rel_ + z_bias_;
        cmd_msg_.position.x = x_act_;
        cmd_msg_.position.y = y_act_;
        cmd_msg_.position.z = z_act_;
    }
}

void msg_transfer::msg_transferNode::consoleCallBack(const arm_test::position::ConstPtr& rel_msg){
    x_rel_ = rel_msg->x_relative;
    y_rel_ = rel_msg->y_relative;
    z_rel_ = rel_msg->z_relative;

    ROS_INFO_STREAM("  x_rel_:   " << x_rel_ << 
                    "      y_rel_:   " << y_rel_ <<
                    "      z_rel_:   " << z_rel_);

    ROS_INFO_STREAM("  x_ref_:   " << x_ref_ << 
                    "      y_ref_:   " << y_ref_ <<
                    "      z_ref_:   " << z_ref_);

    ROS_INFO_STREAM("  x_act_:   " << x_act_ << 
                    "      y_act_:   " << y_act_ <<
                    "      z_act_:   " << z_act_);
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

void msg_transfer::msg_transferNode::odom_cmdPublisher(const ros::TimerEvent& time_event){
    OdometryPublisher_.publish(odom_msg_);
    PositionCommandPublisher_.publish(cmd_msg_);
}

bool msg_transfer::msg_transferNode::setPoint(
                            msg_transfer::SetRefPoint::Request& req_set_point, 
                            msg_transfer::SetRefPoint::Response& res_set_point){
    ROS_INFO_STREAM("Desired pose requested");

    received_desired_pose_ = true;
    x_bias_ = req_set_point.pose.position.x;
    y_bias_ = req_set_point.pose.position.y;
    z_bias_ = req_set_point.pose.position.z;

    ROS_INFO_STREAM("\nx_bias:  " << x_bias_ <<
                    "\ny_bias:  " << y_bias_ <<
                    "\nz_bias:  " << z_bias_);
}