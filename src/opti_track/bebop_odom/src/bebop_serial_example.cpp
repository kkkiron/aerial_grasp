#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <UavLink.h>

#include <iostream>
#include <vector>
using namespace std;

#define rBUFFERSIZE     10
uint8_t r_buffer[rBUFFERSIZE];
vector<uint8_t> vec_serial;
unsigned char usart1_rec_data[256];
int usart1_read_index=0;


int main (int argc, char** argv){

    printHello();
    ros::init(argc, argv, "bebop_serial_example_node");
    ros::NodeHandle nh;
    
    ros::Publisher pub_takeoff = nh.advertise< std_msgs::Empty >("/bebop/takeoff", 1000);
    ros::Publisher pub_land    = nh.advertise< std_msgs::Empty >("/bebop/land", 1000);
    ros::Publisher pub_setpoint = nh.advertise< geometry_msgs::Twist >("/bebop_vel_ctrl/setpoint/cmd_vel", 1000);
    
    std_msgs::Empty msg_takeoff;
    std_msgs::Empty msg_land;
    geometry_msgs::Twist msg_setpoint;
    msg_setpoint.linear.x = 0.0;
    msg_setpoint.linear.y = 0.0;
    msg_setpoint.linear.z = 0.0;
    msg_setpoint.angular.x = 0.0;
    msg_setpoint.angular.y = 0.0;
    msg_setpoint.angular.z = 0.0;
    
    serial::Serial ser;//声明串口对象 

    try
    {
        //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB1");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    
    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    
    //指定循环的频率 
    ros::Rate loop_rate(10);
    while(ros::ok())
    {

        uint8_t a = 0x00;
        unsigned char b = a;
        //cout << b << endl;

        if( ser.read(vec_serial) )
        {
            ROS_INFO("vec_serial");
            ROS_INFO("[0x%02x]",vec_serial.at( vec_serial.size()-1 ));
            //unsigned long a=vec_serial.at( vec_serial.size()-1);
            //unsigned char x;
            //memcpy(&x,&a,sizeof(unsigned long) );
            usart1_rec_data[usart1_read_index] = vec_serial.at( vec_serial.size()-1);
            usart1_read_index++;
//            for(int i=0;i<vec_serial.size();i++)
//                ROS_INFO("[0x%02x]",vec_serial.at(i));
        }


//        if(a = ser.available())
//        {
//            ROS_INFO("ser.available() is %d",a);
//
//            ser.read(r_buffer,rBUFFERSIZE);
//
//            for(int i=0;i<rBUFFERSIZE;i++)
//                ROS_INFO("[0x%02x]",r_buffer[i]);
//
//
//            {
//                pub_takeoff.publish(msg_takeoff);
//
//                pub_land.publish(msg_land);
//
//                msg_setpoint.linear.x = 0.0;
//                msg_setpoint.linear.y = 0.0;
//                msg_setpoint.linear.z = 0.0;
//                msg_setpoint.angular.x = 0.0;
//                msg_setpoint.angular.y = 0.0;
//                msg_setpoint.angular.z = 0.0;
//                pub_setpoint.publish(msg_setpoint);
//            }
//
//        }
        ros::spinOnce();
        loop_rate.sleep();

    }
}


