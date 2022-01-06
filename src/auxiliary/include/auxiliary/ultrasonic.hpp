#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include <iostream>
#include <stdio.h>
#include "serial.h"
#include <unistd.h>
namespace ultra{
    class ultrasonic
    {
    private:
        uint16_t Height;
        uint8_t uls_rec_data[2];
        uint8_t uls_send_data[3];
    public:
        ultrasonic();
        ~ultrasonic();
        void SendData(serial::Serial &ser);
        void RecData(serial::Serial &ser);
        uint16_t GetHeight(serial::Serial &ser,uint8_t ms_10);  
        void TimeTest(serial::Serial &s);
    };
}

#endif
