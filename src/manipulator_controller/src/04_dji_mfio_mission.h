#ifndef MFIO_MISSION_H
#define MFIO_MISSION_H

// System includes
#include "unistd.h"
#include <cstdint>
#include <iostream>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/MFIOConfig.h>
#include <dji_sdk/MFIOSetValue.h>

// ROS includes
#include <ros/ros.h>
#include "std_msgs/Char.h"
#include "std_msgs/UInt32.h"
typedef enum MODE {
  MODE_PWM_OUT  = 0,
  MODE_PWM_IN   = 1,
  MODE_GPIO_OUT = 2,
  MODE_GPIO_IN  = 3,
  MODE_ADC      = 4
} MODE;

typedef enum CHANNEL {
  CHANNEL_0 = 0,
  CHANNEL_1 = 1,
  CHANNEL_2 = 2,
  CHANNEL_3 = 3,
  CHANNEL_4 = 4,
  CHANNEL_5 = 5,
  CHANNEL_6 = 6,
  CHANNEL_7 = 7,
} CHANNEL;

typedef struct ServiceAck
{
  bool         result;
  int          cmd_set;
  int          cmd_id;
  unsigned int ack_data;
  ServiceAck(bool res, int set, int id, unsigned int ack)
    : result(res)
    , cmd_set(set)
    , cmd_id(id)
    , ack_data(ack)
  {
  }
  ServiceAck()
  {
  }
} ServiceAck;

class dji_mfio_ctrl
{
protected:
  ros::NodeHandle nh_;
  ros::ServiceClient      drone_activation_service;
  ros::ServiceClient      mfio_config_service;
  // ros::ServiceClient      mfio_set_value_service;

  ros::Subscriber g_mission;
  ros::Subscriber r_mission;

  ros::Subscriber g_ontime_config;
  ros::Subscriber r_ontime_config;

  uint16_t            pwmFreq_          = 50  ; //Hz
  uint32_t            GraspOnTimeUs_    = 700 ; // us
  uint32_t            ReleaseOnTimeUs_  = 1700 ; // us


public:
  dji_mfio_ctrl(const ros::NodeHandle &nh);
  ~dji_mfio_ctrl();

  void dji_mfio_init();
  ServiceAck dji_mfio_activate();
  void show_instruction();

  void init_slave();

void pwm_release_callback(const std_msgs::Char::ConstPtr msg);
void pwm_grasp_callback(const std_msgs::Char::ConstPtr msg);

void turn_off_pwm();


void r_ontime_config_callback(const std_msgs::UInt32::ConstPtr msg);

void g_ontime_config_callback(const std_msgs::UInt32::ConstPtr msg);

};
#endif // MFIO_MISSION_H
