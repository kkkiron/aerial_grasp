#include "../include/msg_transfer.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "msg_transfer");
    ros::NodeHandle n;
    

    msg_transfer::armtestNode * ArmtestNode = new msg_transfer::armtestNode(n);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    delete ArmtestNode;
    ArmtestNode = nullptr;

    return 0;
}