#include <iostream>
#include <serial.h>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include "../include/auxiliary/auxiliary.hpp"

using namespace std;


void auxiliary ::DataPackage::CtrGripper(){
    if(sta == loose)
        this->package[GRIPPER_CTR] = 0x22; 
    else if(sta == grasp)
       package[GRIPPER_CTR] = 0x11; 
    else if(sta == stop)   //stop 
        package[GRIPPER_CTR] = 0x33;
    // else 
    //     package[GRIPPER_CTR] = 0x00;
}

void auxiliary::DataPackage::SetGripperSta(GripperStatus sta){
    this->sta = sta;
}

void auxiliary ::DataPackage::ClearCtrPackage(){
    for(int i = 0; i < 25; i++)
        package[i] = 0;
}


void auxiliary ::DataPackage::SendControlPackage( serial::Serial &s ){
    
    if(s.isOpen()){
        size_t byte = s.write( package, 25 );
        //DEBUG PRINT
        //for(int i = 0; i < 25; i++)
        //    printf("%x ", this->package[i]);
        //printf("\n");
        if(byte == 25){
            //std::cout<<"Send success"<< std::endl;
        }   
        else{
            std::cout<<"It's send "<< byte << " only."<<std::endl;
        }
    }
    else
        std::cout<<"Usart isn't opening."<<endl;
}

void auxiliary ::DataPackage::ReceiveMsg(serial::Serial &mySerial , auxiliary::arm (&p1)[6]){
    //std::cout<<"Coming ReceiveMsg"<<std::endl;
    if(mySerial.isOpen() == true){
        size_t num = mySerial.available();
        this->rev = new uint8_t[num];
        mySerial.read(rev,num);
        RevSize = num;
        //cout<<"Receive num:"<<num<<endl;
    }
    else{
        std::cout<<"Serial is not opening"<<std::endl;
        return ;
    }

    uint8_t i = 0;
    if (rev[i+1] == 0xAA && rev[i] == 0xAA){
        i += 2;
        while(rev[i]!=~0x44)
        {
            uint8_t ID = rev[i];
            uint16_t pos = ((uint16_t)rev[i+2])<<8 | rev[i+1];
            if(pos <= 1000)
                p1[ID].SetPos(pos);
            i += 3;
        }
    }

    else if (rev[i+1] == 0xBB && rev[i] == 0xBB){
        if(rev[2]==0x11) this->sta=loose;
        else if(rev[2]==0x22) this->sta=grasp;
        else this->sta=stop;
    }
    delete []rev;
}

auxiliary ::DataPackage::DataPackage(){
    for(uint8_t i = 0;  i < 25 ; i++)
        package[i] = 0;
    rev = nullptr;
    sta = dummy;    
}

void auxiliary ::DataPackage::GroupFrames(const auxiliary ::arm (&p1)[6]){
    uint8_t package_header = 0xDD;
    package[FIRST_HEADER] = package[SECOND_HEADER] = package_header;
    package[FIRST_TAIL]   = package[SECOND_TAIL] = ~package_header;
    for(int i = 1; i <= 5; i++){
        uint16_t pos    = p1[i].GetCtrPos();
        uint16_t times  = p1[i].GetCtrTime();
        uint8_t index = 4 * i - 2;
        package[index] = (uint8_t)pos;
        package[index+1] = (uint8_t)(pos>>8);
        package[index+2] = (uint8_t)times;
        package[index+3] = (uint8_t)(times>>8);
    }
    CtrGripper();
}


