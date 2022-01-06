#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from std_msgs.msg import Empty

###########################################
################ callback #################
###########################################
def callback(msg):
    rospy.loginfo(msg)

###########################################
########### publish the topic #############
###########################################
def pub_sub():
    rospy.init_node("sending", anonymous=False)
    
    pub = rospy.Publisher("chatter1", Header, queue_size=10)
    msg = Header()
    
    rospy.Subscriber("chatter2", Header, callback)
    
    rate = rospy.Rate(1) # hz
    rospy.loginfo("OK")
    i = 1
    #rospy.spin()
    while not rospy.is_shutdown():
        msg.seq = i
        i = i+1
        msg.stamp = rospy.get_rostime()
        pub.publish(msg)
        #rospy.loginfo(msg)
        
        
        rate.sleep()

###########################################
##################  main ##################
###########################################
if __name__ == '__main__':
    try:
        pub_sub()
    except rospy.ROSInterruptException:
        pass
