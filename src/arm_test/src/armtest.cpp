#include "../include/arm_test/arm_test.hpp"
#include "std_msgs/Char.h"
arm_test::armtestNode::armtestNode(ros::NodeHandle &n){
    backGround = cv::Mat(30,30,CV_8UC1,cv::Scalar(0));
    
    DisplayName = "arm_test";

    this->InitPublishers(n);
    this->InitPanelThread();
}

void arm_test::armtestNode::InitPublishers(ros::NodeHandle &n){
    // ArmControlPublisher = n.advertise<arm_test::controls>("controls",10);
    ArmControlPublisher = n.advertise<std_msgs::Int8>("mp_cmd",10);
    GripperPublisher    = n.advertise<arm_test::gripper>("gripper",10);
    RefPointPublisher   = n.advertise<arm_test::point>("RefPoint",10);
    PositionPublisher   = n.advertise<arm_test::position>("/position",10);
    TrackPublisher      = n.advertise<arm_test::track>("/track",10);
    _1dof_gripper_r     = n.advertise<std_msgs::Char>("release",10);
    _1dof_gripper_g     = n.advertise<std_msgs::Char>("grasp",10);
}

void arm_test::armtestNode::InitPanelThread(){
    std::thread dat(std::bind(&armtestNode::PanelThread,this));
    dat.detach();
}

void arm_test::armtestNode::PanelThread(){
    ros::Rate LoopRate(10);
    // const std::vector<uint16_t> pos1 {0,  20,  243,   31,   80,  468};
    // const std::vector<uint16_t> tim1 {0,  2000,  2000,  2000,  2000,  2000};

    // const std::vector<uint16_t> pos2 {0,  800,  654,  432,  394,  468};
    // const std::vector<uint16_t> tim2 {0,  2000,  2000,  2000,  2000,  2000};
    //  以上是LX15D机械臂位置
    const std::vector<uint16_t> pos1 {0,  972,   1000,   28,   600,  468};
    const std::vector<uint16_t> tim1 {0,  2000,  2000,  2000,  500,  2000};

    const std::vector<uint16_t> pos2 {0,  972,   1000,   28,   390,  468};
    const std::vector<uint16_t> tim2 {0,  2000,  2000,  2000,  500,  2000};

    const std::vector<uint16_t> pos3 {0,  347,   597,   446,   600,  468};
    const std::vector<uint16_t> tim3 {0,  2000,  2000,  2000,  500,  2000};

    const std::vector<uint16_t> pos4 {0,  347,   597,   446,   390,  468};
    const std::vector<uint16_t> tim4 {0,  2000,  2000,  2000,  500,  2000};

    //2006位置控制


    const std::vector<uint16_t> motor1  {0xAA ,0x00, 0x00, 0xAA, 0x01, 0x00, 0x00, 0x01, 0xff};
    const std::vector<uint16_t> motor2  {0xAA ,0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0xff};

    const std::vector<float>  point1 {0,0,1.6};
    const std::vector<float>  point2 {1,0,1.6};
    const std::vector<float>  point3 {0,1,1.6};
    const std::vector<float>  point4 {1,1,1.6};
    const std::vector<float>  point5 {10,0,1.6};

    const float control_presision = 0.01;

    // arm_test::controls ctr;
    std_msgs::Int8 ctr;
    arm_test::gripper  gr;
    arm_test::point Refp;
    arm_test::position pos;
    arm_test::track is_track;
    float x_r = 0.0, y_r = 0.0, z_r = 0.0;
    bool track = false;
    while(ros::ok()){
        imshow(this->DisplayName, this->backGround);
        // ROS_INFO("release: press q; grasp: press w ");
        ////czj
        
        // std::stringstream ss;
        // ss<<"mission";
        // msg.data = ss.str();
        char k = (char)cv::waitKey(1);
        std_msgs::Char msg;
        msg.data = k;
        switch(k){
                // ros Client 
            /* robotic arm switch */     
            case 'n':
                // //czj
                _1dof_gripper_r.publish(msg);
                break;

            case 'm':
                // //czj
                _1dof_gripper_g.publish(msg);
                break;
            /* robotic arm switch */ 

            /* quadrotor direction */            
            case 'w':
                y_r += control_presision;
                pos.x_relative = x_r;
                pos.y_relative = y_r;
                pos.z_relative = z_r;
                this->PositionPublisher.publish(pos);
                std::cout << "y++: " << y_r << std::endl;
                break; 
            
            case 's':
                y_r -= control_presision;
                pos.x_relative = x_r;
                pos.y_relative = y_r;
                pos.z_relative = z_r;
                this->PositionPublisher.publish(pos);
                std::cout << "y--: " << y_r << std::endl;
                break;
            
            case 'a':
                x_r += control_presision;
                pos.x_relative = x_r;
                pos.y_relative = y_r;
                pos.z_relative = z_r;
                this->PositionPublisher.publish(pos);
                std::cout << "x++: " << x_r << std::endl;
                break; 
             
            case 'd':
                x_r -= control_presision;
                pos.x_relative = x_r;
                pos.y_relative = y_r;
                pos.z_relative = z_r;
                this->PositionPublisher.publish(pos);
                std::cout << "x--: " << x_r << std::endl;
                break;
            
            case 'q':
                z_r += control_presision;
                pos.x_relative = x_r;
                pos.y_relative = y_r;
                pos.z_relative = z_r;
                this->PositionPublisher.publish(pos);
                std::cout << "z++: " << z_r << std::endl;
                break;
            
            case 'e':
                z_r -= control_presision;
                pos.x_relative = x_r;
                pos.y_relative = y_r;
                pos.z_relative = z_r;
                this->PositionPublisher.publish(pos);
                std::cout << "z--: " << z_r << std::endl;
                break;
            
            case 'g':
                x_r = 0;
                y_r = 0;
                z_r = 0;
                pos.x_relative = x_r;
                pos.y_relative = y_r;
                pos.z_relative = z_r;
                this->PositionPublisher.publish(pos);
                std::cout << "set relative distance to zero " << z_r << std::endl;
                break;

            case 't':
                track = !track;
                if(track) {
                    std::cout << "set uav untrack to target " << track << std::endl;
                }
                else{
                    std::cout << "set uav track to target " << track << std::endl;
                }
                is_track.is_track = track;
                this->TrackPublisher.publish(is_track);
                break;
            /* quadrotor direction */

                
            case '1':
                    ctr.data  = 0;
                    this->ArmControlPublisher.publish(ctr);
                    std::cout << "pose 1" << std::endl;
                    break;
            // case '2':
            //         ctr.data  = 1;
            //         this->ArmControlPublisher.publish(ctr);
            //         std::cout << "pose 2" << std::endl;
            //         break;
            case '3':
                    ctr.data  = 2;
                    this->ArmControlPublisher.publish(ctr);
                    std::cout << "pose 3" << std::endl;
                    break;
            case '4':
                    ctr.data  = 3;
                    this->ArmControlPublisher.publish(ctr);
                    std::cout << "pose 4" << std::endl;
                    break;

            case '5':
                    ctr.data  = 96;
                    this->ArmControlPublisher.publish(ctr);
                    std::cout << "griper pose 1" << std::endl;
                    break;

            case '6':
                    ctr.data  = 97;
                    this->ArmControlPublisher.publish(ctr);
                    std::cout << "griper pose 2" << std::endl;
                    break;
            // case 'w':
            //         ctr.armCtr  = pos2;
            //         ctr.timeCtr = tim2;
            //         ctr.GripSta = 0xff;
            //         this->ArmControlPublisher.publish(ctr);
            //         std::cout << "pose 2" << std::endl;
            //         break;
        //     case 'q':
        //             ctr.armCtr  = motor1;
        //             ctr.timeCtr = tim1;
        //             ctr.GripSta = 0xff;
        //             this->ArmControlPublisher.publish(ctr);
        //             std::cout << "pose 1" << std::endl;
        //             break;
        //     case 'w':
        //             ctr.armCtr  = motor2;
        //             ctr.timeCtr = tim2;
        //             ctr.GripSta = 0xff;
        //             this->ArmControlPublisher.publish(ctr);
        //             std::cout << "pose 2" << std::endl;
        //             break;
        //     case 'e':
        //             ctr.armCtr  = pos3;
        //             ctr.timeCtr = tim3;
        //             ctr.GripSta = 0xff;
        //             this->ArmControlPublisher.publish(ctr);
        //             std::cout << "pose 3" << std::endl;
        //             break;
        //     case 'r':
        //             ctr.armCtr  = pos4;
        //             ctr.timeCtr = tim4;
        //             ctr.GripSta = 0xff;
        //             this->ArmControlPublisher.publish(ctr);
        //             std::cout << "pose 4" << std::endl;
        //             break;

        //     case 'b':
        //             gr.GripSta = 0x00;
        //             this->GripperPublisher.publish(gr);
        //             std::cout<<"grasp"<<std::endl;
        //             //std::cout<<"++"<<std::endl;
        //             break;
        //     case 'n':
        //             gr.GripSta = 0x01;
        //             this->GripperPublisher.publish(gr);
        //             std::cout<<"loose"<<std::endl;
        //             //std::cout<<"--"<<std::endl;
        //             break;

        //     case 'm':
        //             gr.GripSta = 0xff;
        //             this->GripperPublisher.publish(gr);
        //             std::cout<<"stop"<<std::endl;
        //             break;
    
        //     case 'l':
        //             gr.GripSta = 0x03;
        //             this->GripperPublisher.publish(gr);
        //             std::cout << "land" << std::endl;
                        

        }
        LoopRate.sleep();
    }
}
