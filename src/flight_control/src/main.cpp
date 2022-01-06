#include <iostream>
#include "../include/flight_control/flightcontrol.hpp"
#include <ros/ros.h>



int main( int argc, char **argv ){
    ros::init(argc, argv, "flight_control");
    ros::NodeHandle n;
    //RoboticArm::RoboticArmNode * myArmNode = new RoboticArm::RoboticArmNode(n);
    flight_control::FlightControlNode *myFlightControlNode = new flight_control::FlightControlNode(n);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    delete myFlightControlNode;
    myFlightControlNode = nullptr;
    return 0;

}
