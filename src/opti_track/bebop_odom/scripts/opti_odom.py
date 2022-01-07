#!/usr/bin/env python3
from NatNetClient import NatNetClient
#import numpy as np
import queue
import math
import rospy
import threading
from operator import mod
from opti_msgs.msg import Odom

total_rigidBody = 4
sampleInterval = 10
s1 = threading.Semaphore(1)

isReceiveNewData = 0
sampleCount = 0
deltaTime = 0
lastTimestamp = 0
###########################################
# optitrack stuff
###########################################

l_odom = [list() for _ in range(total_rigidBody)]
l_index = [-1 for _ in range(total_rigidBody)]
# the meaning of the index is 
# 0: position
# 1: angle
# 2: linearVelocity
# 3: angularVelocity
Position = [0,0,0]
lastPosition = [0,0,0]
Angle = [0,0,0]
lastAngle = [0,0,0]
linearVelocity = [0,0,0]
angularVelocity = [0,0,0]

topicName = 'agent/opti_odom'
nodeName = 'agent_opti_node'

###########################################
######### calculate Euler Angles ##########
###########################################
def quaternion_to_euler_angle(w, x, y, z):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = (math.atan2(t0, t1))
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = (math.asin(t2))
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = ((math.atan2(t3, t4))+2*math.pi)%(2*math.pi)
    return roll, pitch, yaw

###########################################
###### calculate Differential Angles ######
###########################################
def angDiff(ang_end, ang_start):
    ang = ang_end - ang_start
    ang = mod(ang+math.pi,2*math.pi) - math.pi
    return ang
    

###########################################
######### calculate the odometry ##########
###########################################
def receiveRigidBodyFrame(id, pos, rotation):
    if id >= total_rigidBody:
        pass
    #rospy.loginfo(pos)
    id -= 1
    msg = Odom()
    msg.header.frame_id = "inertial"
    msg.rigidBodyID = id
    msg.header.stamp = rospy.get_rostime()
    msg.position.x = pos[0]
    msg.position.y = pos[1]
    msg.position.z = pos[2]
    angle = quaternion_to_euler_angle(rotation[3], rotation[0], rotation[1], rotation[2])
    msg.euler.x = angle[0]
    msg.euler.y = angle[1]
    msg.euler.z = angle[2]
    s1.acquire()
    if len(l_odom[id]) == sampleInterval:
        last_index = (l_index[id] + 2) % sampleInterval
        last_msg = l_odom[id][last_index]
        deltatime = msg.header.stamp - last_msg.header.stamp
        deltatime =  deltatime.secs + deltatime.nsecs/1000000000
        #rospy.loginfo(deltatime)
        msg.linear.x = (pos[0] - last_msg.position.x)/deltatime
        msg.linear.y = (pos[1] - last_msg.position.y)/deltatime
        msg.linear.z = (pos[2] - last_msg.position.z)/deltatime
        if abs(msg.linear.x) < 0.0001:
            msg.linear.x = 0
        if abs(msg.linear.y) < 0.0001:
            msg.linear.y = 0
        msg.angular.x = angDiff(angle[0],last_msg.euler.x)/deltatime
        msg.angular.y = angDiff(angle[1],last_msg.euler.y)/deltatime
        msg.angular.z = angDiff(angle[2],last_msg.euler.z)/deltatime
    else:
        l_odom[id].append(msg)
    l_index[id] = (l_index[id] + 1) % sampleInterval
    l_odom[id][l_index[id]] = msg
    s1.release()

    
###########################################
########### publish the topic #############
###########################################
rospy.init_node(nodeName, anonymous=False)
pub_node = rospy.Publisher(topicName, Odom, queue_size=10)
rate = rospy.Rate(50) # hz
rospy.loginfo("OK")
def pub():
    while not rospy.is_shutdown():
        s1.acquire()
        for i in range(total_rigidBody):
            if l_index[i] == -1:
                continue
            pub_node.publish(l_odom[i][l_index[i]])
         #   print('x:',l_odom[i][l_index[i]].linear.x,'y',l_odom[i][l_index[i]].linear.y)    
        #rospy.loginfo('publish agent%s/opti_odom'%i)
        s1.release()
        rate.sleep()

###########################################
##################  main ##################
###########################################
if __name__ == '__main__':
    #streamingClient = NatNetClient("172.16.6.124") #WIFI
    #streamingClient = NatNetClient("172.16.8.82")  #Net1
    #streamingClient = NatNetClient("172.16.5.205")  #Net2
    #streamingClient = NatNetClient("196.168.199.202")  #Net2
    #streamingClient = NatNetClient("196.168.1.104")  #Net2
    streamingClient = NatNetClient("192.168.1.100")  #Net2
    streamingClient.rigidBodyListener = receiveRigidBodyFrame
    #streamingClient.newFrameListener = receiveNewFrame
    streamingClient.run()
    
    try:
        pub()
    except rospy.ROSInterruptException:
        pass

