#include "../include/arm_test/arm_test.hpp"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "armtest");
    ros::NodeHandle n;
    
    ROS_INFO_STREAM(" \n \\
                    Note of Console Node: \n \\
                    This node used to control uav and robotic arm. \n \\
                    Keys w, a, s, d control uav x, y position, keys q, e control uav z position. \n \\
                    All this x, y, z position are relative distance to desire position of uav. \n \\
                    Before you start the uav, x, y, z must be set to an resonable position by service /set_pose \n \\
                    or key g set to zero. \n \\
                    Keys n, m control robotic arm release or grasp. \n \\
                    Key t controls uav tracks target or not. \n \\
                    ");
    arm_test::armtestNode * ArmtestNode = new arm_test::armtestNode(n);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    delete ArmtestNode;
    ArmtestNode = nullptr;

    return 0;
}