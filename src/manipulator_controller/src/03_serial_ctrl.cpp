#include "03_serial_ctrl.h"

#define BYTE_TO_HW(High, Low) ((((uint16_t)(High)) << 8) | (uint8_t)(Low))  

serial_ctrl::serial_ctrl(const ros::NodeHandle& nh):
nh_(nh)
{
    this->init();
}

void serial_ctrl::init()
{
    lobot_serial_.setPort(port_);
    lobot_serial_.setBaudrate(baud_);
    lobot_serial_.setTimeout(t_);

    serial_buf_sub_ = nh_.subscribe<manipulator_msgs::serial_buf>("mp_serial_buf", 10, &serial_ctrl::serial_write_callback, this); 
    resp_pos_pub_   = nh_.advertise<manipulator_msgs::serial_buf>("mp_resp_buf", 10);
    
    while(!lobot_serial_.isOpen()){
        ROS_WARN("serial port is not open,trying to open");
        lobot_serial_.open();
    }
    ROS_INFO("open serial port successfully!");


}

void serial_ctrl::serial_write_callback(const manipulator_msgs::serial_buf::ConstPtr& msg)
{
    if(msg->buf.size() < 6) return;

    manipulator_msgs::serial_buf resp_buf;
    ros::Rate wait_for_read(100);
    lobot_serial_.write(msg->buf);

    while(msg->buf.size() < 8 )     //receive a read_buf
    {
        wait_for_read.sleep();
        if(lobot_serial_.read(resp_buf.buf, 8) != 0)break;
        
    }
    lobot_serial_.read(256);//clear buf

    /* debug */
    for (size_t i = 0; i < resp_buf.buf.size() && verbal; i++)
    {
        std::cout<<(int)resp_buf.buf[i]<<" ";
    }

    if(resp_buf.buf.size() > 7){
        //in this case 
            //low:  buf[5]
            //high: buf[6]
        uint16_t pos = BYTE_TO_HW(resp_buf.buf[6],resp_buf.buf[5]);
        if(verbal) printf("\n sending resp_buf to protocol_node:  id = %u, pos = %u\n\n",resp_buf.buf[2],pos);
        resp_pos_pub_.publish(resp_buf);
    }
    /* debug */


    return ;
}