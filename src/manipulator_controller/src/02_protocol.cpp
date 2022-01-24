#include "02_protocol.h"

protocol::protocol(const ros::NodeHandle &nh):
    nh_(nh)
{
    this->init();
}

void protocol::init()
{
    memset(pkg_hash_table_, 0xFF, 512);
    memset(dym_hash_table_, 0xFF, 512);

    pos_time_sub_   = nh_.subscribe<manipulator_msgs::id_pos_time>("mp_set_id_pos_time", 10, &protocol::package_callback, this);
    serial_buf_pub_ = nh_.advertise<manipulator_msgs::serial_buf>("mp_serial_buf", 10);

    // resp_buf_sub_   = nh_.subscribe<manipulator_msgs::serial_buf>("mp_resp_buf", 10, &protocol::resp_buf_callback, this);


}

void protocol::package_callback(const manipulator_msgs::id_pos_time::ConstPtr& msg)
{

    if(msg->pos < 0 || msg->pos > 1000)
    {
        ROS_WARN("illegal pos! check again");//当前舵机pos范围是0~1000，不会出现负数
    }
    pkg_hash_table_[msg->ID] = msg->pos;
    set_buf(msg);
    manipulator_msgs::serial_buf move_buf;
    std::vector<uint8_t> move_tmp(move_buf_, move_buf_ + 10);
    move_buf.buf = move_tmp;

    manipulator_msgs::serial_buf read_buf;
    std::vector<uint8_t> read_tmp(read_buf_, read_buf_ + 6);
    read_buf.buf = read_tmp;

    serial_buf_pub_.publish(move_buf);

    // serial_buf_pub_.publish(read_buf);

}

void protocol::set_buf(const manipulator_msgs::id_pos_time::ConstPtr& msg)
{
    id_ = msg->ID, pos_ = msg->pos, time_ = msg->time;

        move_buf_[0] = move_buf_[1] = HEADER;      //header already set while initializing
        move_buf_[2] = id_;
        move_buf_[3] = 7;                                    
        move_buf_[4] = LOBOT_SERVO_MOVE_TIME_WRITE;           //读写标记位，写：0000 0001
                                                        //           读：0000 0010
        move_buf_[5] = GET_LOW_BYTE(pos_);                //位置信息，一共16位
        move_buf_[6] = GET_HIGH_BYTE(pos_);
        move_buf_[7] = GET_LOW_BYTE(time_);
        move_buf_[8] = GET_HIGH_BYTE(time_);
        move_buf_[9] = lobot_checksum(move_buf_);


        read_buf_[0] = read_buf_[1] = HEADER;
        read_buf_[2] = id_;
        read_buf_[3] = 3;
        read_buf_[4] = LOBOT_SERVO_POS_READ;
        read_buf_[5] = lobot_checksum(read_buf_);

}
uint8_t protocol::lobot_checksum(uint8_t buf[])
{
    uint8_t i;
    uint16_t temp = 0;
    for (i = 2; i < buf[3] + 2; i++) {
        temp += buf[i];
    }
    temp = ~temp;
    i = (uint8_t)temp;
    return i;
}

void protocol::resp_buf_callback(const manipulator_msgs::serial_buf::ConstPtr& msg)
{
    uint8_t tmp_id = msg->buf[2];
    uint16_t tmp_pos = BYTE_TO_HW(msg->buf[6],msg->buf[5]);
    uint8_t tmp_read_buf_[6];

    if(tmp_id < 255 && tmp_pos < 1024)
    {
        tmp_read_buf_[0] = tmp_read_buf_[1] = HEADER;
        tmp_read_buf_[2] = tmp_id;
        tmp_read_buf_[3] = 3;
        tmp_read_buf_[4] = LOBOT_SERVO_POS_READ;
        tmp_read_buf_[5] = lobot_checksum(tmp_read_buf_);
        
        dym_hash_table_[tmp_id] = tmp_pos;

        manipulator_msgs::serial_buf read_buf;
        std::vector<uint8_t> read_tmp(tmp_read_buf_, tmp_read_buf_ + 6);
        read_buf.buf = read_tmp;

        ros::Rate wait_desired_pos(50);
        wait_desired_pos.sleep();
        if(simple_abs(tmp_pos, pkg_hash_table_[tmp_id]) < 6)
        {
            if(verbal)ROS_INFO("servo(id: %u) already moved to desire pos!",msg->buf[2]);
            return;
        }
        ROS_INFO("recursively publish read_buf");
        serial_buf_pub_.publish(read_buf);
                
    }

}


/* base protocol class */ 
base_protocol::base_protocol(const ros::NodeHandle &nh):
nh_(nh)
{  
    this->base_init();
}

base_protocol::~base_protocol()
{  
    
}

void base_protocol::base_init()
{

}


/* derived protocol class */ 

derived_protocol::derived_protocol(const ros::NodeHandle &nh):
    base_protocol(nh)
{
    this->derived_init();
}
derived_protocol::~derived_protocol()
{

}


void derived_protocol::derived_init()
{
    memset(pkg_hash_table_, 0xFF, 512);
    memset(dym_hash_table_, 0xFF, 512);

    pos_time_sub_   = nh_.subscribe<manipulator_msgs::id_pos_time>("mp_set_id_pos_time", 10, &derived_protocol::package_callback, this);
    serial_buf_pub_ = nh_.advertise<manipulator_msgs::serial_buf>("mp_serial_buf", 10);
    // resp_buf_sub_   = nh_.subscribe<manipulator_msgs::serial_buf>("mp_resp_buf", 10, &derived_protocol::resp_buf_callback, this);
}

void derived_protocol::package_callback(const manipulator_msgs::id_pos_time::ConstPtr& msg)
{

    if(msg->pos < 0 || msg->pos > 1000)
    {
        ROS_WARN("illegal pos! check again");//当前舵机pos范围是0~1000，不会出现负数
    }
    pkg_hash_table_[msg->ID] = msg->pos;
    set_buf(msg);
    manipulator_msgs::serial_buf move_buf;
    std::vector<uint8_t> move_tmp(move_buf_, move_buf_ + 10);
    move_buf.buf = move_tmp;

    manipulator_msgs::serial_buf read_buf;
    std::vector<uint8_t> read_tmp(read_buf_, read_buf_ + 6);
    read_buf.buf = read_tmp;

    serial_buf_pub_.publish(move_buf);

    // serial_buf_pub_.publish(read_buf);

}

void derived_protocol::set_buf(const manipulator_msgs::id_pos_time::ConstPtr& msg)
{
    id_ = msg->ID, pos_ = msg->pos, time_ = msg->time;

        move_buf_[0] = move_buf_[1] = HEADER;      //header already set while initializing
        move_buf_[2] = id_;
        move_buf_[3] = 7;                                    
        move_buf_[4] = LOBOT_SERVO_MOVE_TIME_WRITE;           //读写标记位，写：0000 0001
                                                        //           读：0000 0010
        move_buf_[5] = GET_LOW_BYTE(pos_);                //位置信息，一共16位
        move_buf_[6] = GET_HIGH_BYTE(pos_);
        move_buf_[7] = GET_LOW_BYTE(time_);
        move_buf_[8] = GET_HIGH_BYTE(time_);
        move_buf_[9] = lobot_checksum(move_buf_);


        read_buf_[0] = read_buf_[1] = HEADER;
        read_buf_[2] = id_;
        read_buf_[3] = 3;
        read_buf_[4] = LOBOT_SERVO_POS_READ;
        read_buf_[5] = lobot_checksum(read_buf_);

}
uint8_t derived_protocol::lobot_checksum(uint8_t buf[])
{
    uint8_t i;
    uint16_t temp = 0;
    for (i = 2; i < buf[3] + 2; i++) {
        temp += buf[i];
    }
    temp = ~temp;
    i = (uint8_t)temp;
    return i;
}

void derived_protocol::resp_buf_callback(const manipulator_msgs::serial_buf::ConstPtr& msg)
{
    uint8_t tmp_id = msg->buf[2];
    uint16_t tmp_pos = BYTE_TO_HW(msg->buf[6],msg->buf[5]);
    uint8_t tmp_read_buf_[6];

    if(tmp_id < 255 && tmp_pos < 1024)
    {
        tmp_read_buf_[0] = tmp_read_buf_[1] = HEADER;
        tmp_read_buf_[2] = tmp_id;
        tmp_read_buf_[3] = 3;
        tmp_read_buf_[4] = LOBOT_SERVO_POS_READ;
        tmp_read_buf_[5] = lobot_checksum(tmp_read_buf_);
        
        dym_hash_table_[tmp_id] = tmp_pos;

        manipulator_msgs::serial_buf read_buf;
        std::vector<uint8_t> read_tmp(tmp_read_buf_, tmp_read_buf_ + 6);
        read_buf.buf = read_tmp;

        ros::Rate wait_desired_pos(50);
        wait_desired_pos.sleep();
        if(simple_abs(tmp_pos, pkg_hash_table_[tmp_id]) < 6)
        {
            if(verbal)ROS_INFO("servo(id: %u) already moved to desire pos!",msg->buf[2]);
            return;
        }
        serial_buf_pub_.publish(read_buf);
                
    }

}
