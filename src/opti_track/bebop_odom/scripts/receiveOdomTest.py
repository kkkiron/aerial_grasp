#!/usr/bin/env python

import rospy
from opti_msgs.msg import Odom

rigidBodyID = 1
neighborID1 = 2
neighborID2 = 3

nodeName = 'agent%s_alg'%rigidBodyID
topicName1 = 'agent%s_odom'%neighborID1
topicName2 = 'agent%s_odom'%neighborID2

odom1 = Odom()
odom2 = Odom()

###########################################
################ callback1 ################
###########################################
def callback1(msg):
    global odom1
    odom1 = msg
    rospy.loginfo("%s %s.%s",neighborID1, odom1.header.stamp.secs, odom1.header.stamp.nsecs)

###########################################
################ callback2 ################
###########################################
def callback2(msg):
    global odom2
    odom2 = msg
    rospy.loginfo("%s %s.%s",neighborID2, odom2.header.stamp.secs, odom2.header.stamp.nsecs) 
    
###########################################
################### sub ###################
###########################################
def sub():
    rospy.init_node(nodeName, anonymous=False)
    
    rospy.Subscriber(topicName1, Odom, callback1)
    rospy.Subscriber(topicName2, Odom, callback2)
    rospy.loginfo("OK")
    rospy.spin()

###########################################
##################  main ##################
###########################################
if __name__ == '__main__':
    try:
        sub()
    except rospy.ROSInterruptException:
        pass
