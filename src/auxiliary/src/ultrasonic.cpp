#include "../include/auxiliary/ultrasonic.hpp"

    void ultra::ultrasonic::SendData(serial::Serial &ser)
    {
        if(ser.isOpen()){
            ser.write(&uls_send_data[0],1);
            usleep(20);
            ser.write(&uls_send_data[1],1);
            usleep(20);
            ser.write(&uls_send_data[2],1);
        }
    }
    
    void ultra::ultrasonic::RecData(serial::Serial &ser)
    {
        uint8_t *rec =new uint8_t[2];
        ser.read(rec,2);
        this->Height=(((uint16_t)rec[0])<<8) + ((uint16_t)rec[1]);
        // printf("H:%d L:%d Hei:%d\n",h,l,Hei);
        // std::cout << std::hex << rec[0] << ' ' << std::hex << rec[1] << std::endl;
        //std::cout << this->Height << "mm" << std::endl;

        //printf("hello\n");
        //printf("H:%d L:%d HEI:%d\n",rec[0],rec[1],this->Height);
        delete[] rec;
    }

    ultra::ultrasonic::ultrasonic()
    {
        this->uls_send_data[0]=0xe8;//address
        this->uls_send_data[1]=0x02;//register
        this->uls_send_data[2]=0xbc;//cmd
        this->Height=0;
    }

    uint16_t ultra::ultrasonic::GetHeight(serial::Serial &s,uint8_t ms_10)
    {
        if(ms_10==0) this->SendData(s);
        if(ms_10==8) this->RecData(s);
        return this->Height;
    }

    ultra::ultrasonic::~ultrasonic()
    {
    }

    void ultra::ultrasonic::TimeTest(serial::Serial &s){
        this->SendData(s);
        usleep(10000);
        this->RecData(s);
    }
