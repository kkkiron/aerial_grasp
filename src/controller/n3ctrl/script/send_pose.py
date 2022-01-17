#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import math
import geometry_msgs.msg
#from gp_algorithm.srv import SetPose
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped
import numpy as np

class send_traj():
    def __init__(self):
        # traj_id_ = 1
        self.point_pub = rospy.Publisher('cmd', PositionCommand, queue_size = 1000)    #发布gp后的位置
        rospy.Subscriber("/traj_start_trigger", PoseStamped, self.traj_start_trigger_callback)
        print("send_pose node is ok.")
    # def send_points(self):
    def traj_start_trigger_callback(self, msg):
        #发送初始pos
        for i in range(50):
            p_0 = np.array([0, 0, 0.5])
            v_0 = np.array([0, 0, 0, 0.0])
            a_0 = np.array([0, 0, 0.0, 0.0])
            pos_init = PositionCommand()

            pos_init.header.stamp = rospy.Time.now()
            pos_init.header.frame_id = "world"

            pos_init.trajectory_flag = 1
            pos_init.trajectory_id = 1

            pos_init.position.x = p_0[0]
            pos_init.position.y = p_0[1]
            pos_init.position.z = p_0[2]

            pos_init.velocity.x = v_0[0]
            pos_init.velocity.y = v_0[1]
            pos_init.velocity.z = v_0[2]

            pos_init.acceleration.x = a_0[0]
            pos_init.acceleration.y = a_0[1]
            pos_init.acceleration.z = a_0[2]

            pos_init.yaw = 0.0
            pos_init.yaw_dot = 0.0

            self.point_pub.publish(pos_init)
            print(pos_init)
        # self.send_pose(pose_init)    #发送
            time.sleep(0.1) #延时0.5s

        # #构建直线点集 
        # pos_t_point = lambda t: np.array([0 * t, 1 * t, 0.5 * np.ones_like(t)])
        # # vel_t = np.array([0, 0, 0])

        # vel_t = np.array([0, 0, 0])
        # # vel_t_point = lambda t: np.array([1 * np.ones_like(t), np.zeros_like(t), np.zeros_like(t)])
        # acc_t = np.array([0, 0, 0])
        # # total_steps = np.linspace(0, 2, 200)
        # total_steps = np.linspace(0, 2, 1000)
        # pos_point = pos_t_point(total_steps)
        # # vel_point = vel_t_point(total_steps)
        # # cols=np.shape(pos_point)[1]
        # # rows=np.shape(pos_point)[0]
        # for i in range(np.shape(pos_point)[1]):
        # # for i in range(5):
        #     pose_goal = PositionCommand()

        #     pose_goal.header.stamp = rospy.Time.now()
        #     pose_goal.header.frame_id = "world"

        #     pose_goal.trajectory_flag = 1
        #     pose_goal.trajectory_id = 1;

        #     pose_goal.position.x = pos_point[0][i]
        #     pose_goal.position.y = pos_point[1][i]
        #     pose_goal.position.z = pos_point[2][i]

        #     pose_goal.velocity.x = vel_t[0]
        #     pose_goal.velocity.y = vel_t[1]
        #     pose_goal.velocity.z = vel_t[2]

        #     pose_goal.acceleration.x = acc_t[0]
        #     pose_goal.acceleration.y = acc_t[1]
        #     pose_goal.acceleration.z = acc_t[2]

        #     pose_goal.yaw = 0.0
        #     pose_goal.yaw_dot = 0.0
        #     print(pose_goal)

        #     self.point_pub.publish(pose_goal)
        #     time.sleep(0.05)
        # #构建直线点集 
        # pos_t_point = lambda t: np.array([0 * t, 1 * t, 1 * np.ones_like(t)])
        # vel_t = np.array([0, 0, 0])
        #构建circular点集 
        # pos_t_point = lambda t: np.array([np.sin(t), 1 + np.cos(t), 0.5+0.18 * t])
        # pos_t_point = lambda t: np.array([np.sin(t), 1 + np.cos(t), 0.5 * np.ones_like(t)])
        # # 构建双扭线
        # # pos_t_point = lambda t: np.array([2 * math.sqrt(np.cos(2*t) * np.cos(t)), 2 * math.sqrt(np.cos(2*t) * np.sin(t)), 0.5 * np.ones_like(t)])
        # # pos_t_point = lambda t: np.array([1.5 * np.cos(0.2 * t), 0.5 + 0.75*np.sin(0.2 * 2 * t), 0.5 * np.ones_like(t)])
        # vel_t = np.array([0, 0, 0])
        # # vel_t_point = lambda t: np.array([1 * np.ones_like(t), np.zeros_like(t), np.zeros_like(t)])
        # acc_t = np.array([0, 0, 0])
        # total_steps = np.linspace(0, 2 * math.pi, 200)
        # # total_steps = np.linspace(0, 2, 10)
        # pos_point = pos_t_point(total_steps)
        # # vel_point = vel_t_point(total_steps)
        # # cols=np.shape(pos_point)[1]
        # # rows=np.shape(pos_point)[0]
        # for i in range(np.shape(pos_point)[1]):
        # # for i in range(5):
        #     pose_goal = PositionCommand()

        #     pose_goal.header.stamp = rospy.Time.now()
        #     pose_goal.header.frame_id = "world"

        #     pose_goal.trajectory_flag = 1
        #     pose_goal.trajectory_id = 2;

        #     pose_goal.position.x = pos_point[0][i]
        #     pose_goal.position.y = pos_point[1][i]
        #     pose_goal.position.z = pos_point[2][i]

        #     pose_goal.velocity.x = vel_t[0]
        #     pose_goal.velocity.y = vel_t[1]
        #     pose_goal.velocity.z = vel_t[2]

        #     pose_goal.acceleration.x = acc_t[0]
        #     pose_goal.acceleration.y = acc_t[1]
        #     pose_goal.acceleration.z = acc_t[2]

        #     pose_goal.yaw = 0.0
        #     pose_goal.yaw_dot = 0.0
        #     print(pose_goal)

        #     file = open('desired_pose.txt',mode = 'a',encoding='utf-8')
        #     file.write(str(pose_goal.position)+'\n')

        #     self.point_pub.publish(pose_goal)
        #     time.sleep(0.05)    #延时50ms


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("send_node")
        s = send_traj()
        #s.send_points()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down send_pose node.")
