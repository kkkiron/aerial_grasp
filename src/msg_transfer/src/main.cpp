#include "../include/msg_transfer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msg_transfer");
    ros::NodeHandle n;
    

    msg_transfer::msg_transferNode * msg_transferNode = new msg_transfer::msg_transferNode(n);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    delete msg_transferNode;
    msg_transferNode = nullptr;

    return 0;
}