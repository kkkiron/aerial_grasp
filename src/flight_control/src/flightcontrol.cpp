#include "../include/flight_control/flightcontrol.hpp"
#include "../include/flight_control/pid.hpp"
#include "../include/flight_control/mpc.hpp"
#include <thread>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

#include <algorithm>
#include "flight_control/state.h"
#include "flight_control/opticalflow.h"

flight_control::FlightControlNode::FlightControlNode(ros::NodeHandle &n)
{

    FlightStatus = 255;

    DisplayMode = 255;

    this->InitSubcribers(n);

    this->InitPublishers(n);

    this->InitServices(n);

    this->InitFlightControlThread();
}

void flight_control::FlightControlNode::InitSubcribers(ros::NodeHandle &n)
{
    LocalPositionSubscriber = n.subscribe<geometry_msgs::PointStamped>("dji_sdk/local_position", 10, &flight_control::FlightControlNode::GetLocalPositionCallBack, this);
    //From radar

    RefPositionSubscriber = n.subscribe<flight_control::point>("RefPoint", 10, &flight_control::FlightControlNode::GetRefPositionCallBack, this);

    MpcOutPutSubscriber = n.subscribe<mav_msgs::RollPitchYawrateThrust>("/cmd_attitude_thrust", 10, &flight_control::FlightControlNode::GetMpcOutPutCallBack, this);

    ThrustCmdSubscriber = n.subscribe<arm_test::gripper>("gripper",10,&flight_control::FlightControlNode::GetThrustCmdCallBack,this);

    DroneArmControlSubscriber = n.subscribe<arm_test::controls>("controls",10,&flight_control::FlightControlNode::GetArmControlCallBack,this);
}

void flight_control::FlightControlNode::InitPublishers(ros::NodeHandle &n)
{
    this->CtrAttitudePublisher = n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
}

void flight_control::FlightControlNode::InitServices(ros::NodeHandle &n)
{
    CtrlAuthorityService = n.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");

    DroneTaskControlService = n.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");

    SetLocalPositionRefService = n.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");

    DroneArmControlService = n.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");
}

void flight_control::FlightControlNode::InitFlightControlThread()
{
    std::thread FlightCtr(std::bind(&flight_control::FlightControlNode::FlightControlThread, this));
    FlightCtr.detach();
}

void flight_control::FlightControlNode::FlightControlThread()
{
    //take off
    //  if (!setLocalPosition())
    //  {
    //      ROS_ERROR("GPS health insufficient");
    //  }

    bool arm_result = set_arm(1);
    if (!arm_result)
    {
        ROS_ERROR("[pilot] fail to arm the flight!!!");
        return;
    }
    
    ros::Duration(2.0).sleep();

    bool ControlAuthority = ObtainControl();
    if(!ControlAuthority){
        ROS_ERROR("Cannot obtain control");
        return ;
    }
    // bool isTakeoff = MonitoredTakeoff();
    // if(!isTakeoff){
    //     ROS_ERROR("Cannot Takeoff");

    // }
    ROS_INFO("Control Start");

    //control
    ros::Rate LoopRate(60);

    while (ros::ok())
    {

        float roll   = u[0] - u_compensate;
        float pitch  = u[1];
        float yaw    = u[2];
        float thrust = u[3] / (3.3 * (9.81 + 20)) * 100;
        std::cout << "     roll: " << roll      << "            pitch: " << pitch << "      thrust:" << thrust <<std::endl;
        // std::cout << "     u_compensate:"  << u_compensate <<  std::endl;
        // std::cout << "     u[o]:"  << u[0] << "     roll:"  << roll << std::endl;
        //float thrust = my_thrust;
        //std::cout << "my_thrust: " <<thrust << std::endl;

        sensor_msgs::Joy controlVelYawRate;
        uint8_t flag = (DJISDK::VERTICAL_THRUST |
                        DJISDK::HORIZONTAL_ANGLE |
                        DJISDK::YAW_RATE   |
                        DJISDK::HORIZONTAL_BODY |
                        DJISDK::STABLE_DISABLE);
        controlVelYawRate.axes.push_back(pitch);   //roll->Vy
        controlVelYawRate.axes.push_back(roll);  //pitch->Vx
        controlVelYawRate.axes.push_back(thrust); //thrust
        controlVelYawRate.axes.push_back(0);      //yaw
        controlVelYawRate.axes.push_back(flag);


        CtrAttitudePublisher.publish(controlVelYawRate);
        // x[0] = localPoint.y;         x[1] = localPoint.x;         x[2] = localPoint.z;
        //std::cout << "localPoint.y: " << localPoint.y << "     " << "localPoint.x: " << localPoint.x  << std::endl;
        // x[3] = HorizontalVelocity.y; x[4] = HorizontalVelocity.x; x[5] = HorizontalVelocity.z;
        // xRef[0] = RefPoint[0];       xRef[1] = RefPoint[1];       xRef[2] = RefPoint[2];
        u_past[0] = u[0];
        u_past[1] = u[1];
        LoopRate.sleep();
    }
    if (TakeoffLand(dji_sdk::DroneTaskControl::Request::TASK_LAND))
        ROS_INFO("Landing success");
}


bool flight_control::FlightControlNode::MonitoredTakeoff()
{

    ros::Time start_time = ros::Time::now();

    if (!TakeoffLand(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
    {
        return false;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();

    // Step 1.1: Spin the motor
    while (FlightStatus != DJISDK::FlightStatus::STATUS_ON_GROUND &&
           DisplayMode != DJISDK::DisplayMode::MODE_ENGINE_START &&
           ros::Time::now() - start_time < ros::Duration(3))
    {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if (ros::Time::now() - start_time > ros::Duration(4))
    {
        ROS_ERROR("Takeoff failed. Motors are not spinnning.");
        return false;
    }
    else
    {
        start_time = ros::Time::now();
        ROS_INFO("Motor Spinning ...");
        ros::spinOnce();
    }

    // Step 1.2: Get in to the air
    while (FlightStatus != DJISDK::FlightStatus::STATUS_IN_AIR &&
           (DisplayMode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || DisplayMode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
           ros::Time::now() - start_time < ros::Duration(2))
    {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if (ros::Time::now() - start_time > ros::Duration(3))
    {



        
        ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
        return false;
    }
    else
    {
        start_time = ros::Time::now();
        ROS_INFO("Ascending...");
        ros::spinOnce();
    }

    // Final check: Finished takeoff
    while ((DisplayMode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || DisplayMode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
           ros::Time::now() - start_time < ros::Duration(2))
    {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if (DisplayMode != DJISDK::DisplayMode::MODE_P_GPS || DisplayMode != DJISDK::DisplayMode::MODE_ATTITUDE)
    {
        ROS_INFO("Successful takeoff!");
        start_time = ros::Time::now();
    }
    else
    {
        ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
        return false;
    }

    return true;
}

bool flight_control::FlightControlNode::ObtainControl()
{

    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 1;
    CtrlAuthorityService.call(authority);

    if (!authority.response.result)
    {
        ROS_ERROR("obtain control failed!");
        return false;
    }

    return true;
}

bool flight_control::FlightControlNode::setLocalPosition()
{
    dji_sdk::SetLocalPosRef localPosReferenceSetter;

    SetLocalPositionRefService.call(localPosReferenceSetter);

    return (bool)localPosReferenceSetter.response.result;
}

bool flight_control::FlightControlNode::TakeoffLand(int task)
{

    dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = task;

    DroneTaskControlService.call(droneTaskControl);

    if (!droneTaskControl.response.result)
    {
        ROS_ERROR("takeoff_land fail");
        return false;
    }

    return true;
}

void flight_control::FlightControlNode::GetLocalPositionCallBack(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    localPoint = msg->point;
    // ROS_INFO("x: %f,y:%f,z:%f", localPoint.x,localPoint.y,localPoint.z);
}



bool flight_control::FlightControlNode::set_arm(int arm)
{
    dji_sdk::DroneArmControl droneArmControl;
    droneArmControl.request.arm = arm;

    DroneArmControlService.call(droneArmControl);
    if (!droneArmControl.response.result)
    {
        ROS_ERROR("[oyzz] arm the flight failed.");
        return false;
    }

    return true;
}

void flight_control::FlightControlNode::GetRefPositionCallBack(const flight_control::point::ConstPtr &msg)
{
    RefPoint = msg->point;
}


void flight_control::FlightControlNode::GetMpcOutPutCallBack(const mav_msgs::RollPitchYawrateThrust::ConstPtr &msg)
{
    this->u[0] = msg->roll;
    this->u[1] = msg->pitch;
    this->u[2] = 0; //yaw = 0
    this->u[3] = msg->thrust.z;
}

void flight_control::FlightControlNode::GetThrustCmdCallBack(const arm_test::gripper::ConstPtr &msg)
{
    if(msg->GripSta==0x00) 
    {
        this->my_thrust++;
        //std::cout << "++" << std::endl;
        //std::cout << "my_thrust: " << my_thrust << std::endl;
    }

    if(msg->GripSta==0x01) 
    {
        this->my_thrust--;
        //std::cout << "--" << std::endl;
        //std::cout << "my_thrust: " << my_thrust << std::endl;
    }
}

void flight_control::FlightControlNode::GetArmControlCallBack(const arm_test::controls::ConstPtr &msg)
{
    this->du = this->max_compensate / 300.0;//(float)(msg->timeCtr[1]);
    // std::cout << "time: " << msg->timeCtr[1] << std::endl;
    std::cout << "du: " << du << std::endl;
    std::cout << "into GetArmControlCallBack function" << std::endl;
    if(msg->armCtr==this->pos4)  //当
    {
        while(u_compensate < max_compensate)  //线性增长至max_compensate
        {
            u_compensate += du;
            ros::Duration(0.001).sleep();
        }

        while(u_compensate >= 0.0)   //减小到0
        {
            u_compensate -= du;
            ros::Duration(0.001).sleep();
        }
    }
    
    else if (msg->armCtr==this->pos5) 
    {
        while(u_compensate > -max_compensate)  
        {
            u_compensate -= du;
            ros::Duration(0.001).sleep();
        }

        while(u_compensate <= 0.0)
        {
            u_compensate += du;
            ros::Duration(0.001).sleep();
        }
    }
}

