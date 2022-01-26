#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import Pose
# from gp_algorithm.srv import SetPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy  
from opti_msgs.msg import Odom  
from n3ctrl.msg import gp_output
from n3ctrl.msg import ControllerDebug
import numpy as np

from IGPR import IGPR

m = 1550.10/1000 #飞机重量
g = 9.8

model_err = 0
state_log = []
controls_log = []
qp_controls_log = []
ref_pos = []
ref_vel = []
windforce_log = []
model_err_log = []
model_err_gp_log = []
model_mse_gp_log = []
model_input_log = []
nearest_list = []
acc_cmd_log = []
kc_log = []
eTPG_log = []
model_err_gp = 0
triggered_log = []
train_time_logs = []

igpr = IGPR(target_counts=3)
steps = 0
pre_update_step = 0

class GP_Algorithm():
    def __init__(self):
        # rospy.init_node('gp_node', anonymous=True)
        #初始化订阅set_pose的xyz
        self.set_pose_x = 0.0
        self.set_pose_y = 0.0
        self.set_pose_z = 0.0

        #初始化订阅/agent/opti_odom的实际xyz
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        
        #初始化rpy
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_rate = 0.0
        self.thrust_x = 0.0
        self.thrust_y = 0.0
        self.thrust_z = 0.0

        

        self.err_pub = rospy.Publisher('con_err_gp', gp_output, queue_size = 10)    #发布gp后的位置
        self.con_pub = rospy.Publisher('uav1/n3ctrl/cmd_gp_out_a', gp_output, queue_size = 10)
        # rospy.Subscriber("n3ctrl/ctrl_dbg/value", ControllerDebug, self.ctrl_callback)    #订阅期望控制量 roll pitch yaw thrust
        rospy.Subscriber("con_to_gp", gp_output, self.ctrl_callback, queue_size = 10)
        rospy.Subscriber("odom", Odometry, self.odom_callback)   #订阅odom 位置 
        rospy.Subscriber("uav1/djiros/imu", Imu, self.imu_callback)   #订阅实际位置、速度
        # rospy.Subscriber("agent/opti_odom", Odom, self.rpy_callback) #订阅实际roll pitch yaw
        
        #输出的期望控制量
    def ctrl_callback(self, req_set):
        self.set_ax = req_set.gp_ax
        self.set_ay = req_set.gp_ay
        self.set_az = req_set.gp_az

        #暂时把algorithm放这里面，待有数据更新时，执行算法
        self.algorithm()

        #optitrack返回的实际位置信息，线速度
    def odom_callback(self, odom):
        self.pose_x = odom.pose.pose.position.x
        self.pose_y = odom.pose.pose.position.y
        self.pose_z = odom.pose.pose.position.z    
        
        self.linear_x = odom.twist.twist.linear.x
        self.linear_y = odom.twist.twist.linear.y
        self.linear_z = odom.twist.twist.linear.z 

        file = open('real_pose.txt',mode = 'a',encoding='utf-8')
        file.write(str(odom.pose.pose.position)+'\n')
        # print('gp1')
        #optitrack返回的实际姿态角
    # def rpy_callback(self, rpy):
    #     self.pitch = rpy.euler.x
    #     self.roll  = rpy.euler.y
    #     self.yaw   = rpy.euler.z
    #     print('gp3')
        #imu返回的实际加速度
    def imu_callback(self, imu):
        self.a_x = imu.linear_acceleration.x
        self.a_y = imu.linear_acceleration.y
        self.a_z = imu.linear_acceleration.z
        # print("imu a_x is %f",self.a_x)
        # print("imu a_y is %f",self.a_y)
        # print("imu a_z is %f",self.a_z)

    def algorithm(self):
        #算法所需的参数经过上面回调函数已更新，这里直接使用
        global model_err
        # global pre_update_step
        # xnew = np.array([self.pose_x, self.pose_y, self.pose_z,
        #                 self.linear_x, self.linear_x, self.linear_x])

        # file = open('data_input.txt',mode = 'a',encoding='utf-8')
        # file.write(str(xnew)+'\n')
        # pred_time = time.time()
        # todo
        # pred_mean = model_err  # igpr.predict(xnew.reshape(1, -1))  #
        # model_err_gp = np.squeeze(pred_mean)       
        # model_err_gp, cov = igpr.predict(xnew.reshape(1, -1))
        # file = open('model_err_gp.txt',mode = 'a',encoding='utf-8')
        # file.write(str(model_err_gp)+'\n')
        # cov = np.sqrt(cov)
        # model_mse_gp = np.squeeze(np.array([cov, cov, cov]))
        # model_err_gp = np.squeeze(np.array(model_err_gp))
        # print(model_err_gp, model_mse_gp)
        # print("pred_time: ", time.time() - pred_time)
        # file = open('pred_time.txt',mode = 'a',encoding='utf-8')
        # file.write(str(time.time() - pred_time)+'\n')
        model_pred = np.array([self.set_ax,self.set_ay,self.set_az])
        #构建发布的消息体
        msg = gp_output()
        msg.header.stamp = rospy.Time.now()
        # msg.gp_ax = 0.02   #gp_output_a_x
        # msg.gp_ay = 0.02
        # msg.gp_az = 0.02
        msg.gp_ax = model_pred[0]   #gp_output_a_x
        msg.gp_ay = model_pred[1]
        msg.gp_az = model_pred[2]
        # msg.thrust.x = 0
        # msg.thrust.y = 0
        # msg.thrust.z = 0
        print('pub con x ', msg.gp_ax)
        print('pub con y ', msg.gp_ay)
        print('pub con z ', msg.gp_az)
        #发布消息
        # print('msg:', msg)
        self.con_pub.publish(msg) 

        # theta = self.pitch  # no consideration of first-order process   pitch
        # phi = self.roll #Roll
   
        # model_input = np.array([self.pose_x, self.pose_y, self.pose_z,
        #                 self.linear_x, self.linear_x, self.linear_x])
        # model first assume psi = 0
        # print('**************')
        # print('set_ax', self.set_ax)
        # print('set_ay', self.set_ay)
        # print('set_az', self.set_az)
        # model_pred = np.array([self.set_ax,self.set_ay,self.set_az])
        # model_pred = - np.array([0, 0, g]) + self.set_Thrust * np.array(
        #         [np.cos(self.set_Roll) * np.sin(self.set_pitch), - np.sin(self.set_Roll), np.cos(self.set_Roll) * np.cos(theta)]) / m
        model_err = np.array([-self.a_x, -self.a_y, self.a_z]) - model_pred  #实际 - 期望
        #构建发布的消息体
        msg = gp_output()
        msg.header.stamp = rospy.Time.now()
        # msg.gp_ax = 0.02   #gp_output_a_x
        # msg.gp_ay = 0.02
        # msg.gp_az = 0.02
        msg.gp_ax = model_err[0]   #gp_output_a_x
        msg.gp_ay = model_err[1]
        msg.gp_az = model_err[2]
        print('cal error x', msg.gp_ax)
        print('cal error y ', msg.gp_ay)
        print('cal error z ', msg.gp_az)
        self.err_pub.publish(msg) 
        # print(model_pred)
        # print('***************************')
        # print(model_err)
        # model_err_log.append(model_err)
        # model_input_log.append(model_input)
       # print("EVENT_TRIGGERED:", EVENT_TRIGGERED, "triggered", sum(triggered) > 0)
        # if True:
            # train_time = time.time()
            # train_data_begin = pre_update_step + 1 if steps - pre_update_step < update_interval else len(model_input_log) - update_interval
            # for i in range(train_data_begin, len(model_input_log)):
            #     gpr.learn(model_input_log[i], model_err_log[i])
            # igpr.learn(model_input, model_err)
            # train_time_logs.append(time.time() - train_time)
            # print("train_time: ", train_time_logs[-1])
            # pre_update_step = steps
        # train_time = time.time()
        # igpr.learn(xnew, model_err)
        # train_time_logs.append(time.time() - train_time)
        # print(model_input,model_err)
        # print("train_time: ", train_time_logs[-1])
        # file = open('train_time.txt',mode = 'a',encoding='utf-8')
        # file.write(str(train_time_logs[-1])+'\n')

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("gp_node")
        GP_Algorithm()
        # gp.algorithm()
        rospy.loginfo("gp_node is starting...")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down GP node.")