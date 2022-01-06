#ifndef _ARMTEST_H
#define _ARMTEST_H

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <arm_test/controls.h>
#include <arm_test/gripper.h>
#include <arm_test/point.h>
#include <arm_test/position.h>
#include <arm_test/track.h>
#include <thread>
#include <opencv2/opencv.hpp>

namespace arm_test{

class armtestNode{
public:
    armtestNode(ros::NodeHandle &n);
    ~armtestNode(){};

    void InitPublishers(ros::NodeHandle &n);
    void InitPanelThread();
    

private:
    ros::Publisher ArmControlPublisher;
    ros::Publisher GripperPublisher;
    ros::Publisher RefPointPublisher;
    ros::Publisher PositionPublisher;
    ros::Publisher TrackPublisher;
    // czj 
    ros::Publisher _1dof_gripper_g;
    ros::Publisher _1dof_gripper_r;
        
    void PanelThread();

    cv::Mat backGround;
    std::string DisplayName;
};



}
#endif
