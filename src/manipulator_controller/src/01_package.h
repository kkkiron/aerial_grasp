#ifndef PACKAGE
#define PACKAGE
#include<ros/ros.h>
#include "id_pos_time.h"

#include<std_msgs/Int8.h>
#include<std_msgs/Int16.h>

#include "02_protocol.h"

class package
{
private:
    /* data */
    bool verbal = false;
    
    ros::NodeHandle nh_;
    ros::Subscriber mp_cmd_sub_;
    ros::Publisher  pos_time_pub_;

    //griper pos time config (griper 即无人机与机械臂的快拆接口，1DoF)
    ros::Subscriber  gp_pos_time_sub_;
    ros::Subscriber  gp_cmd_sub_;




    /* 机械臂姿态数组 */ 
    int16_t mp_pos_arr_[128][5] = {
        /* 姿态1~4， 用于存储1号机械臂的姿态信息 */ 
        {100,   10,     900,   390,  468},
        {100,   10,     900,   600,  468},
        {653,   403,    650,   390,  468},
        {653,   403,    650,   600,  468}
        /* 姿态1~4， 用于存储1号机械臂的姿态信息 */ 



    };
    /* 机械臂姿态数组 */ 


    int16_t time_arr_[4][5]= {
        {999, 999, 999, 999, 999},
        {999, 999, 999, 999, 999},
        {999, 999, 999, 999, 999},
        {999, 999, 999, 999, 999}
    };

    /* 快拆接口姿态数组 */
    int16_t gp_pos_arr_[32][2] = {
        /* 暂时只定义了两个舵机的位置 */
        //                    ID 192↓↓          ID 193↓↓ 
        /* pos0 */ {              735,               500},
        /* pos1 */ {              285,               500},
    /* 快拆接口姿态数组 */

    /* 其余62个ID及姿态信息未扩展 */
    };

    int16_t gp_time_           = 999;


public:
    package(const ros::NodeHandle& nh);
    ~package();

    void init(ros::NodeHandle& nh);

    void arm_callback(const std_msgs::Int8::ConstPtr& msg);

    void arm_ctrl_01(int8_t attitude);

    void config_callback(const std_msgs::Int16::ConstPtr& msg);

    void gp_ctrl_callback(const std_msgs::Int8::ConstPtr& msg);

};

#endif // !PACKAGE




