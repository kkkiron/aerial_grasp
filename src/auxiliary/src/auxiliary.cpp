#include <iostream>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <vector>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "auxiliary/state.h"
#include "auxiliary/controls.h"
#include "../include/auxiliary/auxiliary.hpp"
#include "functional"
#include "../include/auxiliary/ultrasonic.hpp"

#include <chrono>


using namespace std;

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

auxiliary::auxiliaryNode::auxiliaryNode(ros::NodeHandle &n){
    
    myArm[1].SetID(1);
    myArm[1].CtrPos(972);
    myArm[1].CtrTime(900);


    myArm[2].SetID(2);
    //myArm[2].CtrPos(792);
    myArm[2].CtrPos(1000);
    myArm[2].CtrTime(900);

    myArm[3].SetID(3);
    //myArm[3].CtrPos(251);
    myArm[3].CtrPos(28);
    myArm[3].CtrTime(900);
    
    myArm[4].SetID(4);
    //myArm[4].CtrPos(664);
    myArm[4].CtrPos(600);
    myArm[4].CtrTime(900);
   
    myArm[5].SetID(5);
    myArm[5].CtrPos(481);
    myArm[5].CtrTime(900);
    
    this->InitPublishers(n);
    
    this->InitSubcribers(n);
    
    this->InitDataPackageThread(); 
}

void auxiliary::auxiliaryNode::InitPublishers(ros::NodeHandle &n){
    AuxiliaryPublisher = n.advertise<auxiliary::state>("state",10);
    //SetTest(50);
     
    SetPublishFrequency(50);
    std::thread pub(std::bind(&auxiliaryNode::Publish, this));

    pub.detach();    
}

void auxiliary::auxiliaryNode::InitDataPackageThread(){
    std::thread dat(std::bind(&auxiliaryNode::DataPackageThread, this));
    dat.detach();
}



void auxiliary::auxiliaryNode::InitSubcribers(ros::NodeHandle &n){
    ArmControlSubscriber = n.subscribe<auxiliary::controls>
        ("controls",10,&auxiliaryNode::GetArmControlsCallBack,this);
    GripperControlSubscriber = n.subscribe<auxiliary::gripper>
        ("gripper",10,&auxiliaryNode::GripperControlsCallBack,this);
}

// void auxiliary::auxiliaryNode::GetArmControlsCallBack(const auxiliary::controls::ConstPtr& msg){
//     //Get Arm Position controls and time controls
//     std::vector<uint16_t> ArmCtrPos = msg->armCtr;
//     //std::vector<uint16_t> ArmCtrTime = msg->timeCtr;

//     for(int i=1;i<=5;i++){
//         std::cout << "pos:" << ArmCtrPos[i] << ' ';
//     } std::cout << std::endl;
//     // for(int i=1;i<=5;i++){
//     //     std::cout << "tim:" << ArmCtrTime[i] << ' ';
//     // }
//         std::cout << std::endl;  
//     for(uint8_t i = 0; i < 9; i++){
//         this->myPackage.motor2006[i] = ArmCtrPos[i];
//         //this->myArm[i].CtrTime(ArmCtrTime[i]);
//     }
// }

void auxiliary::auxiliaryNode::GetArmControlsCallBack(const auxiliary::controls::ConstPtr& msg){
    //Get Arm Position controls and time controls
    std::vector<uint16_t> ArmCtrPos = msg->armCtr;
    std::vector<uint16_t> ArmCtrTime = msg->timeCtr;

    for(int i=1;i<=5;i++){
        std::cout << "pos:" << ArmCtrPos[i] << ' ';
    } std::cout << std::endl;
    for(int i=1;i<=5;i++){
        std::cout << "tim:" << ArmCtrTime[i] << ' ';
    }
        std::cout << std::endl;  
    for(uint8_t i = 1; i <=5; i++){
        this->myArm[i].CtrPos(ArmCtrPos[i]);
        this->myArm[i].CtrTime(ArmCtrTime[i]);
    }
}

void auxiliary::auxiliaryNode::GripperControlsCallBack(const auxiliary::gripper::ConstPtr& msg){
    //Get Arm Position controls and time controls
    if(msg->GripSta ==  0x00){
        this->myPackage.SetGripperSta(grasp);
        std::cout<<"Grasp"<<std::endl;
    }
    else if(msg->GripSta == 0x01){
        this->myPackage.SetGripperSta(loose);
        std::cout<<"loose"<<std::endl;
    }
    else if(msg->GripSta == 0xff){
        this->myPackage.SetGripperSta(stop);
        std::cout<<"stop"<<std::endl;
    }
}

void auxiliary::auxiliaryNode::DataPackageThread(){
    
    std::string port("/dev/ttyUSB0");
    unsigned long baud= 115200;
    serial::Serial myserial(port, baud, serial::Timeout::simpleTimeout(2)); //block 2ms
    ros::Rate LoopRate(10);

    while(ros::ok()){

        this->myPackage.ReceiveMsg(myserial,this->myArm);

        this->myPackage.GroupFrames(this->myArm);

        this->myPackage.SendControlPackage(myserial);

        LoopRate.sleep();
    }

}



void auxiliary::auxiliaryNode::Publish(){
    auxiliary::state sta;
    ros::Rate LoopRate(PublishFrequency);
    while(ros::ok())
    {
        //sta.height = (float)(this->myPackage.ReturnHeight())/10.0; 
        sta.height =((float)this->height)/1000.0;
        std::vector<uint16_t> ArmPos;
        ArmPos.push_back(0);    //PlaceHolder
        for(uint8_t i = 1; i <=5 ; i++)
        {
            ArmPos.push_back(this->myArm[i].GetPos());
            //printf("The %dth joint is %d\n", i, this->myArm[i].GetPos());
        }
        sta.arm = ArmPos;
        this->AuxiliaryPublisher.publish(sta);
        LoopRate.sleep();
    }

}

void auxiliary::auxiliaryNode::SetPublishFrequency(uint16_t frequency){
    PublishFrequency = frequency;
}



