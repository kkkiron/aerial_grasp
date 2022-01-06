#ifndef _AUXILIARY_H
#define _AUXILIARY_H


#include <serial.h>
#include <v8stdint.h>
#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <vector>
#include <functional>

#include "auxiliary/controls.h"
#include "auxiliary/gripper.h"

#define FIRST_HEADER            0
#define SECOND_HEADER           1
#define FIRST_ARM_POS_CTR_L     2
#define FIRST_ARM_POS_CTR_H     3
#define FIRST_ARM_TIM_CTR_L     4
#define FIRST_ARM_TIM_CTR_H     5
#define SECOND_ARM_POS_CTR_L    6
#define SECOND_ARM_POS_CTR_H    7
#define SECOND_ARM_TIM_CTR_L    8
#define SECOND_ARM_TIM_CTR_H    9
#define THIRD_ARM_POS_CTR_L     10
#define THIRD_ARM_POS_CTR_H     11
#define THIRD_ARM_TIM_CTR_L     12
#define THIRD_ARM_TIM_CTR_H     13
#define FOURTH_ARM_POS_CTR_L    14
#define FOURTH_ARM_POS_CTR_H    15 
#define FOURTH_ARM_TIM_CTR_L    16
#define FOURTH_ARM_TIM_CTR_H    17 
#define FIFTH_ARM_POS_CTR_L     18
#define FIFTH_ARM_POS_CTR_H     19
#define FIFTH_ARM_TIM_CTR_L     20
#define FIFTH_ARM_TIM_CTR_H     21
#define GRIPPER_CTR             22
//#define GIMBAL_YAW_CTR_L        23
//#define GIMBAL_YAW_CTR_H        24
//#define GIMBAL_PITCH_CTR_L      25
//#define GIMBAL_PITCH_CTR_H      26
//#define FB_CTR                  27
#define FIRST_TAIL              23
#define SECOND_TAIL             24


namespace auxiliary {

enum GripperStatus{
    loose = 0,
    grasp = 1,
    stop  = 3,
    dummy
};


class arm{
public:
    //arm(uint8_t num);
    void SetID(uint8_t ID);
    void SetPos(uint16_t pos);
    uint16_t GetPos() const;
    uint8_t GetID() const;
    void CtrPos(uint16_t pos);
    uint16_t GetCtrPos() const;
    void CtrTime( uint16_t set_time );
    uint16_t GetCtrTime() const;

private:
    uint8_t  ID;
    uint16_t ControlPos;
    uint16_t ControlTime;
    uint16_t pos;
};

class DataPackage{
public: 
    //DataPackage(std::string port, unsigned long baud, uint32_t timeout);
    DataPackage();
    void SendControlPackage( serial::Serial &s);                   
    void CtrGripper(); //modify datapackage gripper control bit
    void ClearCtrPackage();
    void GroupFrames(const auxiliary ::arm (&p1)[6]);
    void ReceiveMsg(serial::Serial &mySerial, auxiliary::arm (&p1)[6]);
    serial::Serial myserial;
    void SetGripperSta(GripperStatus sta);
    uint8_t motor2006[9];

private:
    uint8_t package[25];
    uint8_t * rev;
    uint16_t  RevSize; //From one (not zero) to n 
    GripperStatus sta;
};

class auxiliaryNode{
public:
    auxiliaryNode(ros::NodeHandle &n);
    DataPackage myPackage;
    arm myArm[6];
    void InitSubcribers(ros::NodeHandle &n);
    void InitPublishers(ros::NodeHandle &n);
    void InitDataPackageThread();
    void InitOptFlowThread();
    void Publish();
    void DataPackageThread();
    void SetPublishFrequency(uint16_t frequency);


    //void test();
    //void SetTest(int test);
private:
    ros::Publisher AuxiliaryPublisher;
    ros::Publisher OptiFlowPublisher;

    ros::Subscriber ArmControlSubscriber;
    ros::Subscriber GripperControlSubscriber;
    void GetArmControlsCallBack(const auxiliary::controls::ConstPtr& msg);
    void GripperControlsCallBack(const auxiliary::gripper::ConstPtr& msg);
    uint16_t PublishFrequency;
    float height;   // data  get from big ultrasonic
    //std::vector<std::_Bind_helper<false, void (auxiliaryNode::*)(auxiliaryNode *),auxiliaryNode*, auxiliaryNode*&>::type> b;
    //std::thread * PublishThread;
};



}


#endif





