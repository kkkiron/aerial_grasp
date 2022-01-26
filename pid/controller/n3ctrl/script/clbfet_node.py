#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import Pose
# from gp_algorithm.srv import SetPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy  
# from clbfet.msg import Odom  
# from opti_msgs.msg import Odom  
from n3ctrl.msg import gp_output
from adaptive_clbf import AdaptiveClbf
from actionlib_msgs.msg import GoalStatus
# from n3ctrl.msg import ControllerDebug
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

        self.a_x = 0.0
        self.a_y = 0.0
        self.a_z = 0.0
        
        #初始化rpy
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_rate = 0.0
        self.thrust_x = 0.0
        self.thrust_y = 0.0
        self.thrust_z = 0.0

        self.prev_odom_timestamp = rospy.Time(0)

        self.controller_initial()

        self.gp_pub = rospy.Publisher('con_to_gp', gp_output, queue_size = 10)    #发布gp后的位置

        # rospy.Subscriber("n3ctrl/ctrl_dbg/value", ControllerDebug, self.ctrl_callback)    #订阅期望控制量 roll pitch yaw thrust
        rospy.Subscriber("uav1/odom", Odometry, self.odom_callback, queue_size=1)   #订阅odom 位置 
        # rospy.Subscriber("uav1/djiros/imu", Imu, self.imu_callback, queue_size=1)   #订阅实际位置、速度
        rospy.Subscriber("con_err_gp", gp_output, self.imu_callback, queue_size=10)
        # rospy.Subscriber("agent/opti_odom", Odom, self.rpy_callback) #订阅实际roll pitch yaw
        
    def controller_initial(self):
        self.odom_log = []
        self.x_log = []
        self.true_acc_log = []
        self.acc_log = []
        self.predict_error_log = []
        self.predict_var_log = []
        self.true_error_log = []
        self.trigger_log = []
        self.dt_log = []
        self.mu_rm_log = []
        self.mu_pd_log = []
        self.mu_qp_log = []
        self.mu_ad_log = []
        self.sigDelta_log = []
        self.ref_x_log = []
        self.mpc_x_log = []
        self.send_log = []

        self.x_ref = np.zeros((6,1))
        self.x_ref_dot = np.zeros((6,1))

        self.controller = AdaptiveClbf(use_mpc=True, use_trigger=True)
        rospy.loginfo('clbfet controller initial finish')

        params={}
        params['mass'] = 1476/1000
        params['g'] = 9.8

        params["a_lim"] = 13.93
        params["thrust_lim"] = 25#0.5
        params["kp_z"] = 5.0
        params["kd_z"] = 1.0
        params["clf_epsilon"] = 100.0

        params["qp_u_cost"] = 100.0
        params["qp_u_prev_cost"] = 1.0
        params["qp_p1_cost"] = 1.0e8
        params["qp_p2_cost"] = 1.0e12
        params["qp_max_var"] = 1.5
        params["qp_verbose"] = False
        params["max_velocity"] = 2.0
        params["min_velocity"] = 0.5
        params["barrier_vel_gamma"] = 10.0
        params["use_barrier_vel"] = True
        params["use_barrier_pointcloud"] = True
        params["barrier_radius_velocity_scale"] = 0.0
        params["barrier_pc_gamma_p"] = 5.0
        params["barrier_pc_gamma"] = 0.08
        params["verbose"] = False
        params["dt"] = 1.0 / 80.0
        params["max_error"] = 10.0

        params["mpc_stepsize"] = 1
        params["mpc_N"] = 20
        params["mpc_dt"] = 1.0 / 60.0

        # gp params
        params["qp_ksig"] = 1.0e5
        params["measurement_noise"] = 1.0

        # vanilla nn params
        params["qp_ksig"] = 1.0e2
        params["measurement_noise"] = 1.0

        params["N_data"] = 20#600
        params["learning_verbose"] = False
        params["N_updates"] = 50

        
        self.controller.update_params(params)
        # obstacles (x, y, z, radius)
        self.controller.update_barrier_locations(np.array([-100]),np.array([-100]),np.array([-100]),np.array([0.2]))

        self.mass = params['mass']
        self.g = params['g']

        self.dt = params["dt"]
        self.iters = 0
        self.need_send_train = False
        self.sent_train_goal = False
        self.ref_traj = np.zeros((6,20))
        self.add_data = False
        self.use_model = False

        # for cmd
        self.control_mode = -1

        # reference trajectory
        T = 20
        N = int(round(T/self.dt))
        t = np.linspace(0, T-2*self.dt, N-1)
        # self.x_d = np.stack((4 * np.cos(0.2 * t), 2 * np.sin(0.2 * 2 * t), 2 * np.sin(0.2 * 2 * t), np.zeros(N-1), np.zeros(N-1), np.zeros(N-1)))	# 8
        # self.x_d = np.stack((np.ones(N-1)*.7, np.ones(N-1)*.7, np.ones(N-1)*.7, np.zeros(N-1), np.zeros(N-1), np.zeros(N-1)))
        # self.x_d = np.stack((-np.sin(0.5 * t), np.cos(0.5 * t), np.ones(N-1)*.5, np.zeros(N-1), np.zeros(N-1), np.zeros(N-1)))	# circular
        self.x_d = np.stack((np.zeros(N-1), np.zeros(N-1), np.ones(N-1)*.5, np.zeros(N-1), np.zeros(N-1), np.zeros(N-1)))
        # self.x_d[2,-300:] = 0.13
        # line
        # self.x_d = np.stack((np.zeros(N-1), t * .2 - 2, np.ones(N-1)*.5, np.zeros(N-1), np.zeros(N-1), np.zeros(N-1)))
        # self.x_d[1,:800] = 0.
        # self.x_d[1,1200:] = 1
        self.x_d[3,:-1] = np.diff(self.x_d[0,:])
        self.x_d[4,:-1] = np.diff(self.x_d[1,:])
        self.x_d[5,:-1] = np.diff(self.x_d[2,:])
        self.x_d[3,-1]=self.x_d[3,-2]
        self.x_d[4,-1]=self.x_d[4,-2]
        self.x_d[5,-1]=self.x_d[5,-2]
        self.ref_traj = self.x_d[:,self.iters:self.iters+20]
        self.ref_traj = self.x_d[:,:20]

    
    def save_file(self, data, filename, label=[]):
        fp = open('data/'+filename, 'a')
        if len(label) == 0:
            # for d in data:
            fp.write(str(data) + '\n')
        else:
            # for d in data:
            for i in range(len(label)):
                fp.write(label[i] + ': ' + str(data[i]) + '\n')
        fp.close()

    def save_log(self):
        # self.save_file(self.odom_log, 'odom_log.txt', ['px','py','pz','vx','vy','vz'])
        self.save_file(self.x_log[-1], 'x_log.txt', ['px','py','pz','vx','vy','vz'])
        self.save_file(self.true_acc_log[-1], 'true_acc_log.txt', ['ax','ay','az'])
        self.save_file(self.acc_log[-1], 'acc_log.txt', ['ax','ay','az'])
        self.save_file(self.predict_error_log[-1], 'predict_error_log.txt', ['ex','ey','ez'])
        self.save_file(self.predict_var_log[-1], 'predict_var_log.txt', ['vx','vy','vz'])
        self.save_file(self.true_error_log[-1], 'true_error_log.txt', ['ex','ey','ez'])
        self.save_file(self.trigger_log[-1], 'trigger_log.txt')
        self.save_file(self.dt_log[-1], 'dt_log.txt')
        self.save_file(self.mu_rm_log[-1], 'mu_rm_log.txt', ['ux','uy','uz'])
        self.save_file(self.mu_pd_log[-1], 'mu_pd_log.txt', ['ux','uy','uz'])
        self.save_file(self.mu_qp_log[-1], 'mu_qp_log.txt', ['ux','uy','uz'])
        self.save_file(self.mu_ad_log[-1], 'mu_ad_log.txt', ['ux','uy','uz'])
        self.save_file(self.sigDelta_log[-1], 'sigDelta_log.txt', ['vx','vy','vz'])
        self.save_file(self.ref_x_log[-1], 'ref_x_log.txt', ['px','py','pz','vx','vy','vz'])
        self.save_file(self.mpc_x_log[-1], 'mpc_x_log.txt', ['px','py','pz','vx','vy','vz'])
        # self.save_file(self.send_log, 'send_log.txt', ['s1', 's2', 's3', 's4'])

        #输出的期望控制量
    def ctrl_callback(self, req_set):
        self.set_ax = req_set.u_v.x
        self.set_ay = req_set.u_v.y
        self.set_az = req_set.u_v.z

        #暂时把algorithm放这里面，待有数据更新时，执行算法
        # self.algorithm()

        #optitrack返回的实际位置信息，线速度
    def odom_callback(self, odom):
        self.pose_x = odom.pose.pose.position.x
        self.pose_y = odom.pose.pose.position.y
        self.pose_z = odom.pose.pose.position.z    
        
        self.linear_x = odom.twist.twist.linear.x
        self.linear_y = odom.twist.twist.linear.y
        self.linear_z = odom.twist.twist.linear.z 

        
        self.deltatime = odom.header.stamp - self.prev_odom_timestamp
        self.deltatime =  self.deltatime.secs + self.deltatime.nsecs/1000000000

        self.prev_odom_timestamp = odom.header.stamp
        ttt = time.time()
        self.algorithm()
        print(time.time()-ttt)
        # print('z', self.pose_z)

        #optitrack返回的实际姿态角
    # def rpy_callback(self, rpy):
    #     self.pitch = rpy.euler.x
    #     self.roll  = rpy.euler.y
    #     self.yaw   = rpy.euler.z
    #     print('gp3')

        #imu返回的实际加速度
    def imu_callback(self, msg):
        self.a_x = msg.gp_ax
        self.a_y = msg.gp_ay
        self.a_z = msg.gp_az
        # print("imu a_x is %f",self.a_x)
        # print("imu a_y is %f",self.a_y)
        # print("imu a_z is %f",self.a_z)
    
    def algorithm(self):
        if self.sent_train_goal:
            # states 0 = PENDING, 1 = ACTIVE, 2 = PREEMPTED, 3 = SUCCEEDED
            train_state = self.controller.train_model_action_client.get_state()
            # print("State:",train_state)
            if train_state == GoalStatus.SUCCEEDED:
                train_result = self.controller.train_model_action_client.get_result() 
                if hasattr(train_result, 'model_trained'):
                    self.controller.model_trained = self.controller.train_model_action_client.get_result().model_trained
                    self.sent_train_goal = False
                    end_time = rospy.get_rostime()
                    rospy.loginfo('trained')
                    # rospy.logwarn(["training latency (ms): ", (end_time-self.train_start_time).to_sec() * 1000.0])
                # else:
                #     self.controller.model_trained = False
        
        
        x_cur = np.array([[self.pose_x, self.pose_y, self.pose_z,
                        self.linear_x, self.linear_y, self.linear_z]]).T
        # print ('receive x', x_cur.T[0])
        
        self.x_ref_dot = (self.ref_traj[:,:1] - self.x_ref) / self.deltatime
        self.x_ref = self.ref_traj[:,:1]

        true_acc = np.array([-self.a_x, -self.a_y, -self.a_z]).reshape((3,1))
        # print('true acc', true_acc)

        # get acc from controller
        acc = self.controller.get_control(x_cur,self.x_ref,self.x_ref_dot,dt=self.deltatime,use_model=self.use_model,add_data=self.add_data,use_qp=True,ref_traj=self.ref_traj,true_acc=true_acc)

        if acc[0] > 6:
            acc[0] = 6
        if acc[0] < -6:
            acc[0] = -6
        if acc[1] > 6:
            acc[1] = 6
        if acc[1] < -6:
            acc[1] = -6

        self.save_file(x_cur, 'x_log.txt', ['px','py','pz','vx','vy','vz'])
        self.save_file(true_acc.flatten(), 'true_acc_log.txt', ['ax','ay','az'])
        self.save_file(acc, 'acc_log.txt', ['ax','ay','az'])
        self.save_file(self.controller.y_out, 'predict_error_log.txt', ['ex','ey','ez'])
        self.save_file(self.controller.predict_var, 'predict_var_log.txt', ['vx','vy','vz'])
        self.save_file(self.controller.z_dot, 'true_error_log.txt', ['ex','ey','ez'])
        self.save_file(self.controller.qpsolve.triggered, 'trigger_log.txt')
        self.save_file(self.deltatime, 'dt_log.txt')
        self.save_file(self.controller.mu_rm, 'mu_rm_log.txt', ['ux','uy','uz'])
        self.save_file(self.controller.mu_pd, 'mu_pd_log.txt', ['ux','uy','uz'])
        self.save_file(self.controller.mu_qp, 'mu_qp_log.txt', ['ux','uy','uz'])
        self.save_file(self.controller.mu_ad, 'mu_ad_log.txt', ['ux','uy','uz'])
        self.save_file(self.controller.sigDelta, 'sigDelta_log.txt', ['vx','vy','vz'])
        self.save_file(self.x_ref, 'ref_x_log.txt', ['px','py','pz','vx','vy','vz'])
        self.save_file(self.controller.z_ref, 'mpc_x_log.txt', ['px','py','pz','vx','vy','vz'])
        # self.x_log.append(x_cur)
        # self.true_acc_log.append(true_acc.flatten())
        # self.acc_log.append(acc)
        # self.predict_error_log.append(self.controller.y_out)
        # self.predict_var_log.append(self.controller.predict_var)
        # self.true_error_log.append(self.controller.z_dot)
        # self.trigger_log.append(self.controller.qpsolve.triggered)
        # self.dt_log.append(self.deltatime)
        # self.mu_rm_log.append(self.controller.mu_rm)
        # self.mu_pd_log.append(self.controller.mu_pd)
        # self.mu_qp_log.append(self.controller.mu_qp)
        # self.mu_ad_log.append(self.controller.mu_ad)
        # self.sigDelta_log.append(self.controller.sigDelta)
        # self.ref_x_log.append(self.x_ref)
        # self.mpc_x_log.append(self.controller.z_ref)

        if self.iters > 0:
            self.add_data = True
        
        # if self.iters > 640:
        #     self.use_model = True
        
        # if (self.iters - 1 == 20) or self.controller.qpsolve.triggered:
        if self.iters % 40 == 0:
            # self.need_send_train = True
            if not self.sent_train_goal:
                self.train_start_time = rospy.get_rostime()
                print("sending training goal")
                self.controller.train_model_action_client.send_goal(self.controller.train_model_goal)
                self.sent_train_goal = True
        
        # update reference trajectory
        self.iters += 1
        if self.iters < self.x_d.shape[1] - 20:
            self.ref_traj = self.x_d[:,self.iters:self.iters+20]
        
        #构建发布的消息体
        msg = gp_output()
        msg.header.stamp = rospy.Time.now()
        # msg.gp_ax = 0.02   #gp_output_a_x
        # msg.gp_ay = 0.02
        # msg.gp_az = 0.02
        msg.gp_ax = acc[0]   #gp_output_a_x
        msg.gp_ay = acc[1]
        msg.gp_az = acc[2]
        # print('gp_ax ', msg.gp_ax)
        # print('gp_ay ', msg.gp_ay)
        # print('gp_az ', msg.gp_az)
        #发布消息
        # print('msg:', msg)
        self.gp_pub.publish(msg) 

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("clbfet_node")
        GP_Algorithm()
        # gp.algorithm()
        rospy.loginfo("gp_node is starting...")
        rospy.spin()
    except KeyboardInterrupt:

        print("Shutting down GP node.")