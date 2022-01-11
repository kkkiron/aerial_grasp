#ifndef _MSG_TRANSFER_H
#define _MSG_TRANSFER_H

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <opti_msgs/Odom.h>


namespace msg_transfer{

class msg_transferNode{
public:
    msg_transferNode(ros::NodeHandle &n);
    ~msg_transferNode(){};

    void InitPublishers(ros::NodeHandle &n);
    void InitSubscribers(ros::NodeHandle &n);
    

private:
    ros::Publisher OdometryPublisher;
    ros::Publisher PositionCommandPublisher;
    ros::Subscriber OptitrackSubscriber;

    void optitrack_msgCallBack(const opti_msgs::Odom::ConstPtr& opti_msg);
};



}
#endif