#!/usr/bin/env python3
from NatNetClient import NatNetClient
#import numpy as np
import queue
import math
import rospy
import os
import threading
import time
from operator import mod
from opti_msgs.msg import Odom
from geometry_msgs.msg import Twist
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
import rospy


class OptiTrackPublisher:
    def __init__(self, optitrack_ip):
        # parameter
        self.total_rigid_body = 16 #rospy.get_param('/agent_num')
        #print(self.total_rigid_body)
        self.sample_interval = 3
        self.sem = threading.Semaphore(1)

        node_handle = rospy.init_node('agent_opti_node', anonymous=False)

        # list
        # odometry information of each agent
        self.l_odom  = [list() for _ in range(self.total_rigid_body)]
        # l_odom = [ [odom1_{1}, ..., odom1_{sample_interval}], 
        #            ..., 
        #            [odom{total_rigid_body}_{1}, ..., odom{total_rigid_body}_{sample_interval}]  ]

        self.l_stamp = [list() for _ in range(self.total_rigid_body)]
        
        # current odometry index of each agent in l_odom
        self.l_index = [-1 for _ in range(self.total_rigid_body)]
        # [ k1, ..., k{total_rigid_body} ]

        print('l_odom = ', self.l_odom)
        print('l_index = ', self.l_index)

        # OptiTrack
        print('optitrack_ip = ',optitrack_ip)
        self.position_z_init = [-1 for _ in range(self.total_rigid_body)]
        self.streamingClient = NatNetClient(optitrack_ip)  
        self.streamingClient.rigidBodyListListener = self.receiveRigidBodyFrame
        self.streamingClient.run()
        #self.msg2=Odometry

        
    #####################################################################
    ######################## publish the topic ##########################
    #####################################################################
    def pub(self,msg_,id):
        pass
        publishers = {}
        publish_rate = rospy.Rate(100) # hz
        publishers = rospy.Publisher('agent'+str(id)+'/opti_odom', Odometry, queue_size=1)
        publishers.publish(msg_)
        rospy.loginfo(msg_)
        # while not rospy.is_shutdown():
        #     pass
        #     #print('\n\n**********************************************')
        #     for id in range(self.total_rigid_body):
        #         #self.sem.acquire()
        #         if self.l_index[id] == -1:
        #             rospy.logfatal('agent%s no feedback'%id)
        #             continue

        #         # create publisher
        #         if not publishers.__contains__(id):
        #             #publishers[id] = rospy.Publisher('agent'+str(id)+'/opti_odom', Odom, queue_size=1)
        #             publishers[id] = rospy.Publisher('agent'+str(id)+'/opti_odom', Odometry, queue_size=1)
                    

        #         #rospy.loginfo(self.l_odom[id][self.l_index[id]])
        #         publishers[id].publish(msg_)
        #         rospy.loginfo(msg_)
        #         #rospy.loginfo('publish agent%s opti_odom'%id)
        #         #self.sem.release()
            
        #     publish_rate.sleep()
            

    
    #####################################################################
    ###################### calculate the odometry #######################
    #####################################################################
    def receiveRigidBodyFrame(self,rigidBodyList, timestamp):
        #rospy.sleep(0.2)
        #time.sleep(2/10)
        print('receiveRigidBodyFrame')
        #print('receiveRigidBodyFrame tid is %s' % threading.currentThread().ident)
        if rospy.is_shutdown():
            os._exit(0)
        #print('\n\n**********************************************')
        #rospy.loginfo(timestamp)
        for rigidBody in rigidBodyList:
            print(rigidBody)
            id = rigidBody[0]
            print ('checkcheck id ' + str(id))
            pos = rigidBody[1]
            rot = rigidBody[2]
            trackingValid = rigidBody[3]
            if id > self.total_rigid_body:
                continue

            stamp = Time()
            #print(type(msg.header.stamp.secs))
            stamp.data.secs = (int)(timestamp)
            #print(type(msg.header.stamp.nsecs))
            stamp.data.nsecs = int( (timestamp-(int)(timestamp))*1e9 )

            msg = Odom()
            msg.header.frame_id = "0"
            msg.header.stamp = rospy.get_rostime()
            
            
            # optitrack stream id start from 2 
            # update: now start from 1
            id = id - 1
            msg.id = id
            if self.position_z_init[id] == -1:
                self.position_z_init[id] = pos[2]
            msg.position.x = pos[0]
            msg.position.y = pos[1]
            msg.position.z = pos[2] - self.position_z_init[id]
            angle = self.quaternion_to_euler_angle(rot[3], rot[0], rot[1], rot[2])
            msg.euler.x = angle[0]
            msg.euler.y = angle[1]
            msg.euler.z = angle[2]
            #rospy.loginfo(msg)

            msg2=Odometry()
            msg2.header.frame_id = str(id)
            msg2.header.stamp = rospy.get_rostime()
            msg2.pose.pose.position.x = pos[0]
            msg2.pose.pose.position.y = pos[1]
            msg2.pose.pose.position.z = pos[2] - self.position_z_init[id]
            angle = self.quaternion_to_euler_angle(rot[3], rot[0], rot[1], rot[2])
            msg2.pose.pose.orientation.x = angle[0]
            msg2.pose.pose.orientation.y = angle[1]
            msg2.pose.pose.orientation.z = angle[2]
            #rospy.loginfo(msg2)
            self.pub(msg2,id)

            #self.sem.acquire()
            # if len(self.l_odom[id]) == self.sample_interval:
            #     last_index = (self.l_index[id] + 2) % self.sample_interval
            #     last_msg = self.l_odom[id][last_index]
            #     last_stamp = self.l_stamp[id][last_index]

            #     deltatime = stamp.data - last_stamp.data
            #     deltatime =  deltatime.secs + deltatime.nsecs/1e9

            #     msg.deltaTime = deltatime
            #     #rospy.loginfo(deltatime)
            #     msg.linear.x = (msg.position.x - last_msg.position.x)/deltatime
            #     msg.linear.y = (msg.position.y - last_msg.position.y)/deltatime
            #     msg.linear.z = (msg.position.z - last_msg.position.z)/deltatime
            #     #if abs(msg.linear.x) < 0.0001:
            #     #    msg.linear.x = 0
            #     #if abs(msg.linear.y) < 0.0001:
            #     #    msg.linear.y = 0
            #     msg.angular.x = self.ang_diff(angle[0],last_msg.euler.x)/deltatime
            #     msg.angular.y = self.ang_diff(angle[1],last_msg.euler.y)/deltatime
            #     msg.angular.z = self.ang_diff(angle[2],last_msg.euler.z)/deltatime
                
            # else:
            #     self.l_odom[id].append(msg)
            #     self.l_stamp[id].append(stamp)
            # self.l_index[id] = (self.l_index[id] + 1) % self.sample_interval
            # self.l_odom[id][self.l_index[id]] = msg
            # self.l_stamp[id][self.l_index[id]] = stamp
            #self.sem.release()
    
    #####################################################################
    ###################### calculate Euler Angles #######################
    #####################################################################
    def quaternion_to_euler_angle(self, w, x, y, z):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = (math.atan2(t0, t1))
        roll = self.ang_diff(roll, 0)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = (math.asin(t2))
        pitch = self.ang_diff(pitch, 0)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = ((math.atan2(t3, t4))+2*math.pi)%(2*math.pi)
        yaw = self.ang_diff(yaw, 0)
        return roll, pitch, yaw

    #####################################################################
    ################### calculate Differential Angles ###################
    #####################################################################
    def ang_diff(self, ang_end, ang_start):
        ang = ang_end - ang_start
        ang = mod(ang+math.pi,2*math.pi) - math.pi
        return ang














#####################################################################
###############################  main ###############################
#####################################################################
if __name__ == '__main__':
    
    #print('main tid is %s' % threading.currentThread().ident)
    
    optitrack_ip = '172.16.5.205'    # Ethernet
    # optitrack_ip = '172.16.6.124'    # WiFi
    op = OptiTrackPublisher(optitrack_ip)
    


    try:
        print('start')
        #op.pub()
    except rospy.ROSInterruptException:
        pass

