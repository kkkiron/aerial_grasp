#ifndef PROTOCOL
#define PROTOCOL

#include<ros/ros.h>
#include<std_msgs/UInt32.h>
#include<std_msgs/UInt8MultiArray.h>

#include "id_pos_time.h"
#include "serial_buf.h"
#include <memory.h>

#define GET_LOW_BYTE(A) ((uint8_t)(A))
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
#define BYTE_TO_HW(High, Low) ((((uint16_t)(High)) << 8) | (uint8_t)(Low))  

const uint8_t HEADER                            = 0x55;
//
const uint8_t LOBOT_SERVO_MOVE_TIME_WRITE       = 1;
const uint8_t LOBOT_SERVO_POS_READ        = 28;


class protocol
{      
friend class serial_controller;

private:
    /* data */
    bool verbal = false;

    uint16_t pkg_hash_table_[256];
    uint16_t dym_hash_table_[256];

    uint8_t move_buf_[10]    = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t read_buf_[6]    = {0, 0, 0, 0, 0, 0};

    uint8_t id_         = 254;
    int16_t pos_        = 0;
    uint16_t time_      = 0;
    
    /* ros */
    ros::NodeHandle     nh_;
    ros::Subscriber     pos_time_sub_;
    ros::Publisher      serial_buf_pub_;
    
    ros::Subscriber     resp_buf_sub_;

public:
    protocol(const ros::NodeHandle &nh);
    void init();

    void package_callback(const manipulator_msgs::id_pos_time::ConstPtr& msg);
    void set_buf(const manipulator_msgs::id_pos_time::ConstPtr& msg);
    uint8_t lobot_checksum(uint8_t buf[10]);

    void resp_buf_callback(const manipulator_msgs::serial_buf::ConstPtr& msg);
    uint16_t simple_abs(uint16_t a, uint16_t b)
    {
        return a > b ? a - b : b - a;
    }
};


class base_protocol
{
protected:
    /* data */
    bool verbal = false;

    uint16_t pkg_hash_table_[256];
    uint16_t dym_hash_table_[256];

    uint8_t move_buf_[10]    = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t read_buf_[6]    = {0, 0, 0, 0, 0, 0};

    uint8_t id_         = 254;
    int16_t pos_        = 0;
    uint16_t time_      = 0;
    
    /* ros */
    ros::NodeHandle     nh_;

    ros::Subscriber     pos_time_sub_;
    ros::Publisher      serial_buf_pub_;
    ros::Subscriber     resp_buf_sub_;
public:
    base_protocol(const ros::NodeHandle &nh);
    ~base_protocol();

    void base_init();

    virtual void package_callback(const manipulator_msgs::id_pos_time::ConstPtr& msg) = 0;
    virtual void resp_buf_callback(const manipulator_msgs::serial_buf::ConstPtr& msg) = 0;
    virtual void set_buf(const manipulator_msgs::id_pos_time::ConstPtr& msg)          = 0;
};


class derived_protocol : public base_protocol
{
private:
    /* data */

public:
    derived_protocol(const ros::NodeHandle &nh);

    ~derived_protocol();

    void derived_init();

    void package_callback(const manipulator_msgs::id_pos_time::ConstPtr& msg)     override;
    void resp_buf_callback(const manipulator_msgs::serial_buf::ConstPtr& msg)     override;
    void set_buf(const manipulator_msgs::id_pos_time::ConstPtr& msg)              override;


    uint8_t lobot_checksum(uint8_t buf[10]);
    uint16_t simple_abs(uint16_t a, uint16_t b)
    {
        return a > b ? a - b : b - a;
    }

    };




#endif // !PROTOCOL
