
// #include <dji_sdk_demo/demo_mfio.h>

// 【czjnote】include路径为相对路径，需要指明当前功能包文件夹名
//        功能包名        功能包/include文件夹下的hpp文件
#include"ball_releasing/mfio_mission.h"
#include<iostream>
#include "std_msgs/Char.h"
// global variables


/*  实例化3个Client（客户端）  */ 
// 激活无人机的client
ros::ServiceClient      drone_activation_service;

// 配置mfio的client


ros::ServiceClient      mfio_config_service;

// mfio参数设置的client
ros::ServiceClient      mfio_set_value_service;

void pwm_release_callback(const std_msgs::Char& msg){
    ROS_INFO("releasing");
    int responseTimeout = 1;

    // Set SDK1 to output PWM at 50Hz
    uint16_t pwmFreq = 50; //Hz
    // release对应的pwm信号为1100us
    uint32_t initOnTimeUs = 700; // us
    
    ROS_INFO("Configuring channel");
    // 实例化一个MFIConfig类，实例化对象名为mfio_config
    dji_sdk::MFIOConfig mfio_config;

    // 初始化MFIConfig对象参数
    mfio_config.request.mode = MODE::MODE_PWM_OUT;

    // 【czjnote】【czjQ】CHANNEL_0对应的是MFIO的F3
    mfio_config.request.channel = CHANNEL::CHANNEL_0;

    mfio_config.request.init_on_time_us = initOnTimeUs;
    mfio_config.request.pwm_freq = pwmFreq;

    mfio_config_service.call(mfio_config);
    ROS_INFO("Channel configured to output PWM with %d ms /20000ms.",initOnTimeUs);
    // 根据实际情况设定
    sleep(2);
}

void pwm_grasp_callback(const std_msgs::Char& msg){
    ROS_INFO("grasp");
    int responseTimeout = 1;

    // Set SDK1 to output PWM at 50Hz
    uint16_t pwmFreq = 50; //Hz
    // release对应的pwm信号为1100us
    uint32_t initOnTimeUs = 1700; // us
    
    ROS_INFO("Configuring channel");
    // 实例化一个MFIConfig类，实例化对象名为mfio_config
    dji_sdk::MFIOConfig mfio_config;

    // 初始化MFIConfig对象参数
    mfio_config.request.mode = MODE::MODE_PWM_OUT;

    // 【czjnote】【czjQ】CHANNEL_0对应的是MFIO的F3
    mfio_config.request.channel = CHANNEL::CHANNEL_0;

    mfio_config.request.init_on_time_us = initOnTimeUs;
    mfio_config.request.pwm_freq = pwmFreq;

    mfio_config_service.call(mfio_config);
    ROS_INFO("Channel configured to output PWM with %d ms /20000ms.",initOnTimeUs);
    // 根据实际情况设定
    sleep(2);
}

int
main(int argc, char** argv)
{
    // 初始化一个ros节点，名称：ball_releasing
    ros::init(argc, argv, "ball_releasing");

    // 创建控制句柄
    ros::NodeHandle nh;

    // ROS stuff
    // drone_activation_service客户端接收的service消息类型：dji_sdk::Activation
    drone_activation_service  = nh.serviceClient<dji_sdk::Activation>
        ("dji_sdk/activation");//service消息实体：dji_sdk/activation

    // 以下同理
    mfio_config_service       = nh.serviceClient<dji_sdk::MFIOConfig>
        ("dji_sdk/mfio_config");
    mfio_set_value_service    = nh.serviceClient<dji_sdk::MFIOSetValue>
        ("dji_sdk/mfio_set_value");

    // 检查激活情况，激活成功则继续
    if (activate().result) {
        ROS_INFO("Activated successfully");
    } 
    // 否则撤销任务，返回-1
    else {
        ROS_WARN("Failed activation");
        return -1;
    }

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

    // Subscriber节点实现
    ros::Subscriber g_mission;
    g_mission = nh.subscribe("grasp", 10, pwm_grasp_callback);
    
    ros::Subscriber r_mission;
    r_mission = nh.subscribe("release", 10, pwm_release_callback);
    
    // // pwm信号发生函数
    // std_msgs::Char c;
    // for(int i = 0 ; i < 10 ;i++){
    //     pwm_release_callback(c);
    //     sleep(2);
    //     pwm_grasp_callback(c);
    // }
    
    // pwm_func();

    /*除了主程序以外，ROS还会自动在后台按照你规定的格式，接受订阅的消息。
    但是所接到的消息并不是立刻就被处理
    必须要等到ros::spin()或ros::spinOnce()执行的时候才被调用*/ 

    ros::spin();

    return 0;
}

bool pwm_func() {

    int responseTimeout = 1;

    // Set SDK1 to output PWM at 50Hz
    uint16_t pwmFreq = 50; //Hz

    // 设定初始占空比，用于机构复位，根据舵机文档以及设计要求确定
    uint32_t initOnTimeUs = 820; // us
    


    // //根据确定，是否需要预留复位时间
    
/*configuring*/
    // Setup the channel
    ROS_INFO("Configuring channel");
    // 实例化一个MFIConfig类，实例化对象名为mfio_config
    dji_sdk::MFIOConfig mfio_config;

    // 初始化MFIConfig对象参数
    mfio_config.request.mode = MODE::MODE_PWM_OUT;

    // 【czjnote】【czjQ】CHANNEL_0对应的是MFIO的F3？
    mfio_config.request.channel = CHANNEL::CHANNEL_0;

    // 初始化pwm的两个关键参数：1. 占空比； 2. 频率（50Hz，即周期20ms）
    // 更多关于周期的信息见对应舵机文档
    mfio_config.request.init_on_time_us = initOnTimeUs;
    mfio_config.request.pwm_freq = pwmFreq;


    mfio_config_service.call(mfio_config);
    ROS_INFO("Channel configured to output PWM with %d ms /20000ms.",initOnTimeUs);
    // 根据实际情况设定
    sleep(2);
  

    // while(true){
    //     std::cout<<"please input sth……"<<std::endl;
    //     std::cin>>initOnTimeUs;
    //     sleep(0.5);
    //     ROS_INFO("setting output PWM with %d ms /20000ms.",initOnTimeUs);
    
    //     //接下来所有pwm参数设置，均通过mfio_set_value对象实现
    //     dji_sdk::MFIOSetValue mfio_set_value;
    //     mfio_set_value.request.channel = CHANNEL::CHANNEL_0;
    //     mfio_set_value.request.init_on_time_us = initOnTimeUs;

    //     //回调函数传入mfio_set_value对象即可完成设置
    //     mfio_set_value_service.call(mfio_set_value);
    //     sleep(3);
    // }


    // //舵机运行至第2工位的示例
    // initOnTimeUs = 1444; //us, 7.22 percent duty cycle
    // ROS_INFO("setting output PWM with %d ms /20000ms.",initOnTimeUs);
    // mfio_set_value.request.init_on_time_us = initOnTimeUs;
    // mfio_set_value_service.call(mfio_set_value);
    // ROS_INFO("done！");
    // sleep(2);

    //
    // for(int i = 0; i < 10; i++){

    //     initOnTimeUs = 2333; //us, 11.67 percent duty cycle
    //     ROS_INFO("setting output PWM with %d ms /20000ms.",initOnTimeUs);
    //     mfio_set_value.request.init_on_time_us = initOnTimeUs;
    //     mfio_set_value_service.call(mfio_set_value);
    //     sleep(1);
    //     ROS_INFO("done！");

    //     initOnTimeUs = 1444; //us, 7.22 percent duty cycle
    //     ROS_INFO("setting output PWM with %d ms /20000ms.",initOnTimeUs);
    //     mfio_set_value.request.init_on_time_us = initOnTimeUs;
    //     mfio_set_value_service.call(mfio_set_value);
    //     sleep(1);
    //     ROS_INFO("done！");


    // }


    ROS_INFO("Turning off the PWM signal");
    uint32_t digitalValue = 0;
    uint16_t digitalFreq = 0; //Does not matter for digital I/O
    mfio_config.request.mode = MODE::MODE_GPIO_OUT;
    mfio_config.request.channel = CHANNEL::CHANNEL_0;
    mfio_config.request.init_on_time_us = digitalValue;
    mfio_config.request.pwm_freq = digitalFreq;
    mfio_config_service.call(mfio_config);

    return true;
}

ServiceAck
activate()
{
    // 实例化一个Activation类，实例化对象名为activation
    dji_sdk::Activation activation;

    // service回调，传入activation对象
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