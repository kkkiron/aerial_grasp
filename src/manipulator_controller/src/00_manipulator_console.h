#ifndef MP_CONSOLE
#define MP_CONSOLE
#include<ros/ros.h>
#include "id_pos_time.h"

#include<std_msgs/Int8.h>
#include<std_msgs/Int16.h>
enum CMD
{
    /* 1号机械臂 */ 
    MP1_FOLD_GRASP       = 0,
    MP1_FOLD_RELEASE        ,
    MP1_EXTEND_GRASP        ,
    MP1_EXTEND_RELEASE      ,
    /* 1号机械臂 */ 

    MP_GRIPER_ATTACH    = 96,
    MP_GRIPER_DETACH    = 97,

    
};

class mp_console
{
private:
    ros::Subscriber mp_cmd_sub_;
    ros::Publisher  arm_cmd_pub_;
    ros::Publisher  gp_cmd_pub_;
    ros::NodeHandle nh_;
public:
    mp_console(const ros::NodeHandle& nh);
    ~mp_console();
    void init(ros::NodeHandle& nh)
    {
        mp_cmd_sub_     = nh.subscribe("mp_cmd", 10, &mp_console::mp_mission_dispatcher_callback, this);
        arm_cmd_pub_    = nh.advertise<std_msgs::Int8>("arm_cmd", 10);
        gp_cmd_pub_     = nh.advertise<std_msgs::Int8>("gp_cmd", 10);
    }
    void mp_mission_dispatcher_callback(const std_msgs::Int8::ConstPtr& msg)
    {
        ROS_INFO("running dispatcher!");
        std_msgs::Int8  cmd;
        cmd.data = msg->data;
        if(cmd.data >=0 && cmd.data <= 95)
        {
            arm_cmd_pub_.publish(cmd);
        }
        else if(cmd.data >=96 && cmd.data <= 127)
        {
            gp_cmd_pub_.publish(cmd);
        }
        else
        {
            ROS_WARN("wrong cmd, check again!(desired cmd: 0~127 )");
        }


    }

};

mp_console::mp_console(const ros::NodeHandle& nh):
nh_(nh)
{
    this->init(nh_);
}

mp_console::~mp_console()
{
}
#endif // !MP_CONSOLE
 