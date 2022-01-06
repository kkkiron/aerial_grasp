#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H

//ros include
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

//DJI include 
#include "../../../../devel/include/dji_sdk/DroneTaskControl.h"
#include "../../../../devel/include/dji_sdk/SDKControlAuthority.h"
#include "../../../../devel/include/dji_sdk/SetLocalPosRef.h"
#include "../../../dji_sdk/include/dji_sdk/dji_sdk.h"
#include <dji_sdk/DroneArmControl.h>

#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include "flight_control/state.h"
#include "flight_control/opticalflow.h"
#include "flight_control/mpcinputs.h"
#include "flight_control/point.h"
#include "arm_test/gripper.h"
#include "arm_test/controls.h"

#include <mav_msgs/RollPitchYawrateThrust.h>

namespace flight_control{

class FlightControlNode{

public:
    FlightControlNode(ros::NodeHandle &n);
    void InitSubcribers(ros::NodeHandle &n);
    void InitPublishers(ros::NodeHandle &n);
    void InitServices(ros::NodeHandle &n);
    void InitFlightControlThread();
    void FlightControlThread();   //run flight control algorithm 

    bool ObtainControl();
    bool MonitoredTakeoff();
    bool TakeoffLand(int task);

private:
    ros::Subscriber LocalPositionSubscriber;
    ros::Subscriber RefPositionSubscriber;
    ros::Subscriber MpcOutPutSubscriber;
    ros::Subscriber ThrustCmdSubscriber;
    ros::Subscriber DroneArmControlSubscriber;

    ros::Publisher CtrAttitudePublisher;

    ros::ServiceClient CtrlAuthorityService;
    ros::ServiceClient DroneTaskControlService;
    ros::ServiceClient SetLocalPositionRefService;
    ros::ServiceClient DroneArmControlService;

    void GetLocalPositionCallBack(const geometry_msgs::PointStamped::ConstPtr& msg);
    void GetRefPositionCallBack(const flight_control::point::ConstPtr& msg);
    void GetMpcOutPutCallBack(const mav_msgs::RollPitchYawrateThrust::ConstPtr& msg);
    // void GetThrustCmdCallBack(const arm_test::gripper::ConstPtr& msg);
    void GetArmControlCallBack(const arm_test::controls::ConstPtr& msg);

    bool setLocalPosition();
    bool set_arm(int arm);


    geometry_msgs::Point localPoint;
    std::vector<float> RefPoint_ = {0, 0, 1};
    std::vector<float> u_ = {0, 0, 0, 0};
    std::vector<float> temp_u_ = {0, 0, 0, 0};





    //The height of UAV
    double my_thrust_=1;
    const std::vector<uint16_t> pos1 {0,  972,   1000,   28,   600,  468};
    const std::vector<uint16_t> pos3 {0,  347,   597,   446,   600,  468};
    const std::vector<uint16_t> pos4 {0xAA ,0x00, 0x00, 0xAA, 0x01, 0x00, 0x00, 0x01, 0xff};
    const std::vector<uint16_t> pos5 {0xAA ,0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0xff};
    float u_compensate = 0;
    const float max_compensate = 0.01744 * 7.0;
    float du;

    // cmdfliter
    bool is_recepted_ = false;
    bool is_flitered_ = false;
    const int window_size_ = 8;
    int max_index_ = 0, min_index_ = 0; 
    void CmdFliter();
    std::vector<float> gaussian_param_ = {0.0098, 0.0332, 0.088, 0.1838, 0.2996, 0.383};
    std::vector<float> cmd_roll_vector_;
    std::vector<float> cmd_pitch_vector_;
    std::vector<float> cmd_thrust_vector_;
    void FindMinMax(const std::vector<float>& nums);
    float Weighting(const std::vector<float>& nums);
    void Update(std::vector<float>& nums, float new_data);
    void Process(std::vector<float>& nums, float new_data, float& res);
};

}

#endif
