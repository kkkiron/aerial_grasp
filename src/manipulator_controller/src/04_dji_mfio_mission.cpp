#include"04_dji_mfio_mission.h"


dji_mfio_ctrl::dji_mfio_ctrl(const ros::NodeHandle &nh):
    nh_(nh)
{
    this->dji_mfio_init();
    if(this->dji_mfio_activate().result)
    {
        ROS_INFO("Activated successfully");
    }
    else
    {
        ROS_WARN("Failed activation");
        return;
    }
    this->show_instruction();
    this->init_slave();

}

dji_mfio_ctrl::~dji_mfio_ctrl()
{
    this->turn_off_pwm();
}

ServiceAck dji_mfio_activate();


void dji_mfio_ctrl::dji_mfio_init()
{
    drone_activation_service  = nh_.serviceClient<dji_sdk::Activation>   ("dji_sdk/activation");
    mfio_config_service       = nh_.serviceClient<dji_sdk::MFIOConfig>   ("dji_sdk/mfio_config");
    // mfio_set_value_service    = nh_.serviceClient<dji_sdk::MFIOSetValue> ("dji_sdk/mfio_set_value");
}
ServiceAck dji_mfio_ctrl::dji_mfio_activate()
{
    dji_sdk::Activation activation;
    drone_activation_service.call(activation);

    // 激活失败，则抛出警告
    if(!activation.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set, activation.response.cmd_id);
        ROS_WARN("ack.data: %i", activation.response.ack_data);
    }

    //ServAck数据结构包含如下4部分：
        // 1. bool类型的result，标记命令是否执行成功
        // 2. 命令集合
        // 3. 命令id
        // 4. ack数据
    return {activation.response.result, activation.response.cmd_set,
            activation.response.cmd_id, activation.response.ack_data};
}
void dji_mfio_ctrl::show_instruction()
{
    // 显示API使用指引，根据实际情况写
    ROS_INFO("To run the ball_releasing_demo API, please first do the following steps:");
    ROS_INFO("1. Open DJI Assistant 2 and go to the Tools page.");
    ROS_INFO("2. Click on the Function Channels tab.");

    // 根据实际情况选channel、SDK序号
    ROS_INFO("3. Select SDK1 for channel F3.");

    // 本试验直接接舵机:F3口先引出Gnd，Signal信号线
    ROS_INFO("4. Connect a logic analyzer/oscilloscope to this channel.");

    ROS_INFO("   The pin diagram, from top to bottom, is Gnd, 5V, Signal.");

    // 本试验中:舵机与F3口共地；引出的信号线接到舵机信号线；舵机接5V电源
    ROS_INFO("   You will only need to connect to the Gnd and Signal pins.");
}

void dji_mfio_ctrl::init_slave()
{
    r_mission       = nh_.subscribe("release",    10, &dji_mfio_ctrl::pwm_release_callback,    this);
    g_mission       = nh_.subscribe("grasp",      10, &dji_mfio_ctrl::pwm_grasp_callback,      this);

    g_ontime_config = nh_.subscribe("config_r_time",    10, &dji_mfio_ctrl::r_ontime_config_callback,    this);
    r_ontime_config = nh_.subscribe("config_g_time",    10, &dji_mfio_ctrl::g_ontime_config_callback,    this);

}

void dji_mfio_ctrl::pwm_release_callback(const std_msgs::Char::ConstPtr msg){
    ROS_INFO("releasing");
    dji_sdk::MFIOConfig mfio_config_;
    mfio_config_.request.mode = MODE::MODE_PWM_OUT;
    mfio_config_.request.channel = CHANNEL::CHANNEL_0;
    mfio_config_.request.init_on_time_us = GraspOnTimeUs_;
    mfio_config_.request.pwm_freq = pwmFreq_;

    mfio_config_service.call(mfio_config_);
    ROS_INFO("Channel configured to output PWM with %d ms /20000ms.",GraspOnTimeUs_);

}

void dji_mfio_ctrl::pwm_grasp_callback(const std_msgs::Char::ConstPtr msg){
    ROS_INFO("grasp");
    dji_sdk::MFIOConfig mfio_config_;
    mfio_config_.request.mode = MODE::MODE_PWM_OUT;
    mfio_config_.request.channel = CHANNEL::CHANNEL_0;
    mfio_config_.request.init_on_time_us = ReleaseOnTimeUs_;
    mfio_config_.request.pwm_freq = pwmFreq_;

    mfio_config_service.call(mfio_config_);
    ROS_INFO("Channel configured to output PWM with %d ms /20000ms.",ReleaseOnTimeUs_);
}

void dji_mfio_ctrl::turn_off_pwm()
{
    ROS_INFO("Turning off the PWM signal");
    dji_sdk::MFIOConfig mfio_config_;
    uint32_t digitalValue = 0;
    uint16_t digitalFreq = 0; //Does not matter for digital I/O
    mfio_config_.request.mode = MODE::MODE_GPIO_OUT;
    mfio_config_.request.channel = CHANNEL::CHANNEL_0;
    mfio_config_.request.init_on_time_us = digitalValue;
    mfio_config_.request.pwm_freq = digitalFreq;
    mfio_config_service.call(mfio_config_);
}

void dji_mfio_ctrl::r_ontime_config_callback(const std_msgs::UInt32::ConstPtr msg){
    uint32_t val = msg->data;
    if(val < 400 || val > 2200)
    {
        ROS_WARN("wrong pwm signal, please check!");
        return;
    }
    GraspOnTimeUs_ = val;
}

void dji_mfio_ctrl::g_ontime_config_callback(const std_msgs::UInt32::ConstPtr msg){
    uint32_t val = msg->data;
    if(val < 400 || val > 2200)
    {
        ROS_WARN("wrong pwm signal, please check!");
        return;
    }
    ReleaseOnTimeUs_ = val;
}

