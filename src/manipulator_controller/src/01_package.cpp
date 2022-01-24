#include "01_package.h"
package::package(const ros::NodeHandle& nh):
nh_(nh)
{
    this->init(nh_);
}

package::~package(){}

void package::init(ros::NodeHandle& nh)
{
    mp_cmd_sub_             = nh.subscribe("arm_cmd", 10, &package::arm_callback, this);
    pos_time_pub_           = nh.advertise<manipulator_msgs::id_pos_time>("mp_set_id_pos_time", 10);

    gp_cmd_sub_             = nh.subscribe("gp_cmd", 10, &package::gp_ctrl_callback, this);
    gp_pos_time_sub_        = nh.subscribe("gp_pos_time_cfg", 10, &package::config_callback, this);

}

void package::arm_callback(const std_msgs::Int8::ConstPtr& msg)
{
    package::arm_ctrl_01(msg->data);
}

void package::arm_ctrl_01(int8_t attitude)
{
    manipulator_msgs::id_pos_time msg;

    if(attitude <= 3 && attitude >= 0)//姿态模式，根据数组 mp_pos_arr_[] 预置的参数确定四种姿态
    {
        for (size_t i = 0; i < 4; i++)
        {
            msg.CMD = LOBOT_SERVO_MOVE_TIME_WRITE;
            msg.ID = i + 1;
            msg.pos = mp_pos_arr_[attitude][i];
            msg.time = time_arr_[attitude][i];
            // ROS_INFO("ID = %d, POS = %d", i+1, msg.pos);
            pos_time_pub_.publish(msg);
        }
    }
    else
    {

        ROS_INFO("attitue is %d, this is a cmd for arm", attitude);
        ROS_WARN("wrong cmd(we only set arm attitude 1, 2, 3, 4 currently), please check!");
    }

}
void package::config_callback(const std_msgs::Int16::ConstPtr& cfg)
{
    manipulator_msgs::id_pos_time msg;

    int16_t gp_release_pos_    = cfg->data;
    int16_t gp_grasp_pos_      = cfg->data;

    if(cfg->data > 500)//config r_pos             
    {
        int16_t r_pos = 0;
        msg.CMD                             = LOBOT_SERVO_MOVE_TIME_WRITE;
        msg.pos                             = gp_release_pos_;
        msg.time                            = gp_time_;

        msg.ID                              = 192;
        gp_pos_arr_[r_pos][msg.ID - 192]    = gp_release_pos_;
        pos_time_pub_.publish(msg);

        msg.ID                              = 193;
        gp_pos_arr_[r_pos][msg.ID - 192]    = gp_release_pos_;
        pos_time_pub_.publish(msg);
    }
    else if(cfg->data >= 0 && cfg->data < 500)//config g_pos
    {
        int16_t g_pos = 1;
        msg.CMD                             = LOBOT_SERVO_MOVE_TIME_WRITE;
        msg.pos                             = gp_grasp_pos_;
        msg.time                            = gp_time_;

        msg.ID                              = 192;
        gp_pos_arr_[g_pos][msg.ID - 192]    = gp_grasp_pos_;
        pos_time_pub_.publish(msg);

        msg.ID                              = 193;
        gp_pos_arr_[g_pos][msg.ID - 192]    = gp_grasp_pos_;
        pos_time_pub_.publish(msg);

    }
    else if(cfg->data >= 1000 && cfg->data < 100000)//config gp_time
    {
        gp_time_ = cfg->data;
    }
}

void package::gp_ctrl_callback(const std_msgs::Int8::ConstPtr& attitude)
{
    manipulator_msgs::id_pos_time msg;

    if(attitude->data == 96 || attitude->data == 97)
    {
        for (size_t i = 0; i < 2; i++)
        {
            msg.CMD = LOBOT_SERVO_MOVE_TIME_WRITE;
            msg.ID = i + 192;
            msg.pos = gp_pos_arr_[attitude->data - 96][i]; 
            msg.time = gp_time_;
            pos_time_pub_.publish(msg);
        }
    }
    else
    {
        ROS_INFO("attitue is %d, this is a cmd for griper", attitude->data);
        ROS_WARN("wrong cmd(we only set griper attitude 96, 97 currently), please check!");
    }
}
