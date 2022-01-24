#include<memory.h>

#include "00_manipulator_console.h"
#include "01_package.cpp"
#include "02_protocol.cpp"
#include "03_serial_ctrl.cpp"
#include "04_dji_mfio_mission.cpp"

int main(int argc, char **argv)
{
    /* data for all nodes */
    ros::init(argc, argv, "manipulator");
    ros::NodeHandle     nh;

    mp_console*         console_node             = new mp_console(nh);
    derived_protocol*   protocol_node            = new derived_protocol(nh);
    package*            pacakge_node             = new package(nh);
    serial_ctrl*        serial_ctrl_node         = new serial_ctrl(nh);

    // ROS_INFO("wait 3s, then init dji_mfio_ctrl_node");
    // ros::Duration(3).sleep();

    // dji_mfio_ctrl*      dji_mfio_ctrl_node       = new dji_mfio_ctrl(nh);

    ros::spin();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();


    delete console_node;
    console_node = NULL;

    delete protocol_node;
    protocol_node = NULL;

    delete pacakge_node;
    pacakge_node = NULL;

    delete serial_ctrl_node;
    serial_ctrl_node = NULL;

    // delete dji_mfio_ctrl_node;
    // serial_ctrl_node = NULL;
    // delete serial_controller_node;
    // serial_controller_node = NULL;

    return 0;
}
