#include "ros/ros.h"
#include "std_msgs/String.h"
#include "opti_msgs/Odom.h"
#include <iostream>
#include <string>
#include <sstream>

#define rigidBodyID  1
#define neighborID1  2
#define neighborID2  3

opti_msgs::Odom odom1;
opti_msgs::Odom odom2;

/************************************************/
void Callback1(const opti_msgs::Odom::ConstPtr& msg)
{
  odom1 = *msg;
  ROS_INFO_STREAM(odom1.header.stamp); 
}
/************************************************/
void Callback2(const opti_msgs::Odom::ConstPtr& msg)
{
  odom2 = *msg;
  ROS_INFO_STREAM(odom2.header.stamp);
}
/************************************************/
int main(int argc, char **argv)
{
  std::stringstream nodeName;
  nodeName<<"agent"<<rigidBodyID<<"_alg";
  
  std::stringstream topicName1;
  topicName1<<"agent"<<neighborID1<<"_odom";
  ROS_INFO_STREAM("agent"<<neighborID1<<"_odom");
  //std::stringstream topicName2;
  //topicName2<<"agent"<<neighborID2<<"_odom";
  std::string topicName2;
  topicName2 = std::string("agent")+std::to_string(neighborID2)+std::string("_odom");
  
  ros::init(argc, argv, nodeName.str());
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe(topicName1.str(), 100, Callback1);
  ros::Subscriber sub2 = n.subscribe(topicName2, 100, Callback2);
  
  
  
  ros::spin();

  return 0;
}
