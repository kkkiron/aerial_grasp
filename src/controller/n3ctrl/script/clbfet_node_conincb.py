#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import numpy as np
import math
import threading

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from adaptive_clbf import AdaptiveClbf
from actionlib_msgs.msg import GoalStatus
from n3ctrl.msg import gp_output

class CLBFET():
    def __init__(self):
        # get from odom
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        # get from imu
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0

        # data_log
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
        params['mass'] = 1550.10/1000
        params['g'] = 9.8

        params["a_lim"] = 13.93
        params["thrust_lim"] = 20#0.5
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
        params["dt"] = 1.0 / 120.0
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
        self.controller.update_barrier_locations(np.array([0]),np.array([1]),np.array([0.5]),np.array([0.2]))

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

        self.prev_odom_timestamp = rospy.Time(0)
        self.prev_goal_timestamp = rospy.Time(0)

        rospy.Subscriber("agent0/opti_odom", Odometry, self.odom_cb)
        # rospy.Subscriber("model_train", String, self.model_train_cb)
        self.control_pub = rospy.Publisher('', gp_output, queue_size=10)
        

        # reference trajectory
        T = 10
        N = int(round(T/self.dt))
        t = np.linspace(0, T-2*self.dt, N-1)
        # self.x_d = np.stack((4 * np.cos(0.2 * t), 2 * np.sin(0.2 * 2 * t), 2 * np.sin(0.2 * 2 * t), np.zeros(N-1), np.zeros(N-1), np.zeros(N-1)))	# 8
        # self.x_d = np.stack((np.ones(N-1)*.7, np.ones(N-1)*.7, np.ones(N-1)*.7, np.zeros(N-1), np.zeros(N-1), np.zeros(N-1)))
        # self.x_d = np.stack((-np.sin(0.5 * t), np.cos(0.5 * t), np.ones(N-1)*.5, np.zeros(N-1), np.zeros(N-1), np.zeros(N-1)))	# circular
        # self.x_d = np.stack((np.zeros(N-1), np.zeros(N-1), np.ones(N-1)*.5, np.zeros(N-1), np.zeros(N-1), np.zeros(N-1)))
        # self.x_d[2,-300:] = 0.13
        # line
        self.x_d = np.stack((np.zeros(N-1), t * .6 - 2., np.ones(N-1)*.5, np.zeros(N-1), np.zeros(N-1), np.zeros(N-1)))
        self.x_d[1,:400] = 0.
        self.x_d[1,800:] = 2.
        self.x_d[3,:-1] = np.diff(self.x_d[0,:])
        self.x_d[4,:-1] = np.diff(self.x_d[1,:])
        self.x_d[5,:-1] = np.diff(self.x_d[2,:])
        self.x_d[3,-1]=self.x_d[3,-2]
        self.x_d[4,-1]=self.x_d[4,-2]
        self.x_d[5,-1]=self.x_d[5,-2]
        self.ref_traj = self.x_d[:,self.iters:self.iters+20]
        self.ref_traj = self.x_d[:,:20]

        self.save_x = 0.0
        self.save_y = 0.0
        
                
        # self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_cb)
        # TODO

    
    def save_file(self, data, filename, label=[]):
        fp = open(filename, 'w+')
        if len(label) == 0:
            for d in data:
                fp.write(str(d) + '\n')
        else:
            for d in data:
                for i in range(len(label)):
                    fp.write(label[i] + ': ' + str(d[i]) + '\n')
        fp.close()

    def save_log(self):
        self.save_file(self.odom_log, 'odom_log.txt', ['px','py','pz','vx','vy','vz'])
        self.save_file(self.x_log, 'x_log.txt', ['px','py','pz','vx','vy','vz'])
        self.save_file(self.true_acc_log, 'true_acc_log.txt', ['ax','ay','az'])
        self.save_file(self.acc_log, 'acc_log.txt', ['ax','ay','az'])
        self.save_file(self.predict_error_log, 'predict_error_log.txt', ['ex','ey','ez'])
        self.save_file(self.predict_var_log, 'predict_var_log.txt', ['vx','vy','vz'])
        self.save_file(self.true_error_log, 'true_error_log.txt', ['ex','ey','ez'])
        self.save_file(self.trigger_log, 'trigger_log.txt')
        self.save_file(self.dt_log, 'dt_log.txt')
        self.save_file(self.mu_rm_log, 'mu_rm_log.txt', ['ux','uy','uz'])
        self.save_file(self.mu_pd_log, 'mu_pd_log.txt', ['ux','uy','uz'])
        self.save_file(self.mu_qp_log, 'mu_qp_log.txt', ['ux','uy','uz'])
        self.save_file(self.mu_ad_log, 'mu_ad_log.txt', ['ux','uy','uz'])
        self.save_file(self.sigDelta_log, 'sigDelta_log.txt', ['vx','vy','vz'])
        self.save_file(self.ref_x_log, 'ref_x_log.txt', ['px','py','pz','vx','vy','vz'])
        self.save_file(self.mpc_x_log, 'mpc_x_log.txt', ['px','py','pz','vx','vy','vz'])
        self.save_file(self.send_log, 'send_log.txt', ['s1', 's2', 's3', 's4'])

    
    def timer_cb(self, _event):
        if self.need_send_train:
            self.need_send_train = False
            if not self.sent_train_goal:
                self.train_start_time = rospy.get_rostime()
                print("sending training goal")
                self.controller.train_model_action_client.send_goal(self.controller.train_model_goal)
                self.sent_train_goal = True
                print('sending cost', (rospy.get_rostime() - self.train_start_time).to_sec())
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

    def odom_cb(self, odom):
        # TODO:  theory - maybe dt of rostime and header does not match dt of actual odometry data.  this might cause big problems.
        dt = (odom.header.stamp - self.prev_odom_timestamp).to_sec()
        deltatime = odom.header.stamp - self.prev_odom_timestamp
        deltatime =  deltatime.secs + deltatime.nsecs/1000000000
        # dt = 0.02

        self.pose_x = odom.pose.pose.position.x
        self.pose_y = odom.pose.pose.position.y
        self.pose_z = odom.pose.pose.position.z
        if len(self.odom_log) == 0:
            self.linear_x = 0.
            self.linear_y = 0.
            self.linear_z = 0.
        else:
            self.linear_x = (self.pose_x - self.odom_log[-1][0]) / dt
            self.linear_y = (self.pose_y - self.odom_log[-1][1]) / dt
            self.linear_z = (self.pose_z - self.odom_log[-1][2]) / dt
        
        if len(self.odom_log) > 3:
            self.acc_x = (self.linear_x - self.odom_log[-1][3]) / dt
            self.acc_y = (self.linear_y - self.odom_log[-1][4]) / dt
            self.acc_z = (self.linear_z - self.odom_log[-1][5]) / dt

        self.odom_log.append([self.pose_x, self.pose_y, self.pose_z, self.linear_x, self.linear_y, self.linear_z])
        # print ([self.pose_x, self.pose_y, self.pose_z, self.linear_x, self.linear_y, self.linear_z])

        # # print 'iters: ', self.iters

        if dt < 0:
            rospy.logwarn("detected jump back in time!  resetting prev_odom_timestamp.")
            self.prev_odom_timestamp = self.odom.header.stamp
            self.sent_train_goal = False
            return

        # if dt < self.dt:
        #     # rospy.logwarn("dt is too small! (%f)  skipping this odometry callback!", dt)
        #     return
        
        self.prev_odom_timestamp = odom.header.stamp

        # enter of the algorithm
        if not self.control_mode == 2:
            return

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
        print ('receive x', x_cur.T[0])
        
        self.x_ref_dot = (self.ref_traj[:,:1] - self.x_ref) / dt
        self.x_ref = self.ref_traj[:,:1]

        true_acc = np.array([self.acc_x, self.acc_y, self.acc_z]).reshape((3,1))

        # get acc from controller
        acc = self.controller.get_control(x_cur,self.x_ref,self.x_ref_dot,dt=dt,use_model=self.use_model,add_data=self.add_data,use_qp=True,ref_traj=self.ref_traj,true_acc=true_acc)

        self.x_log.append(x_cur)
        self.true_acc_log.append(true_acc.flatten())
        self.acc_log.append(acc)
        self.predict_error_log.append(self.controller.y_out)
        self.predict_var_log.append(self.controller.predict_var)
        self.true_error_log.append(self.controller.z_dot)
        self.trigger_log.append(self.controller.qpsolve.triggered)
        self.dt_log.append(dt)
        self.mu_rm_log.append(self.controller.mu_rm)
        self.mu_pd_log.append(self.controller.mu_pd)
        self.mu_qp_log.append(self.controller.mu_qp)
        self.mu_ad_log.append(self.controller.mu_ad)
        self.sigDelta_log.append(self.controller.sigDelta)
        self.ref_x_log.append(self.x_ref)
        self.mpc_x_log.append(self.controller.z_ref)

        if self.iters > 0:
            self.add_data = True
        
        if (self.iters - 1 == 20) or self.controller.qpsolve.triggered:
        # if self.iters % 40 == 0:
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
        self.control_pub.publish(msg)

        # control crazyflie
        [x_real, y_real, z_real] = [self.pose_x, self.pose_y, self.pose_z]
        self.cf.extpos.send_extpos(x_real, y_real, z_real)
        # convert acc to (thrust, roll, pitch, yaw)
        c = self.controller.dyn.convert_mu_to_control(acc.reshape((3,1)))
        # send control to crazyflie
        self.cf.commander.send_setpoint(np.rad2deg(c[2]), -np.rad2deg(c[1]), 0, int(round(c[0] / self.g * 65536 / 0.06)))

        self.send_log.append([np.rad2deg(c[2]), -np.rad2deg(c[1]), 0, int(round(c[0] / self.g * 65536 / 0.06))])

    
    def model_train_cb(self, key):
        if self.sent_train_goal:
            if key.data == '1':
                self.controller.model_trained = True
                end_time = rospy.get_rostime()
                rospy.loginfo('trained after' + str((end_time-self.train_start_time).to_sec()))
            elif key.data == '0':
                rospy.loginfo('train failed')
            self.sent_train_goal = False


if __name__ == '__main__':
    try:
        rospy.init_node("clbfet_node")
        clbfet = CLBFET()
        rospy.loginfo("clbfet_node is starting...")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down clbfet node.")
