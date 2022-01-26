#ifndef SERIAL_CTRL
#define SERIAL_CTRL

#include<ros/ros.h>
#include<serial.h>
#include<memory.h>

#include<std_msgs/Int16.h>
#include<std_msgs/String.h>
#include<serial_buf.h>

class serial_ctrl
{
private:
    bool verbal = false;
    // /* ros */
    ros::NodeHandle     nh_;
    ros::Subscriber     serial_buf_sub_;
    ros::Publisher      resp_pos_pub_;

    /* serial */
    std::string port_   =    "/dev/ttyUSB1";
    uint32_t baud_      =    115200;
    serial::Timeout t_  =    serial::Timeout::simpleTimeout(2);
    serial::Serial lobot_serial_;

public:
    serial_ctrl(const ros::NodeHandle& nh);
    void init();
    void serial_write_callback(const manipulator_msgs::serial_buf::ConstPtr& msg);

};


#endif // !SERIAL_CTR