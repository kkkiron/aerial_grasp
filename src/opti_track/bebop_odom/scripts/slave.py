#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from std_msgs.msg import Empty

###########################################
################ callback #################
###########################################
def callback(msg):
    global pub
    pub.publish(msg)
    rospy.loginfo(msg)

###########################################
########### publish the topic #############
###########################################
def pub_sub():
    global pub
    
    rospy.init_node("receiving", anonymous=False)
    
    pub = rospy.Publisher("chatter2", Header, queue_size=10)
    
    rospy.Subscriber("chatter1", Header, callback)
    
    rospy.loginfo("OK")
    while not rospy.is_shutdown():
        pass

###########################################
##################  main ##################
###########################################
if __name__ == '__main__':
    
    try:
        pub_sub()
    except rospy.ROSInterruptException:
        pass
