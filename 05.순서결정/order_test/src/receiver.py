#!/usr/bin/env python


import rospy 
from std_msgs.msg import String 
import time

prev_time = time.time()


def callback(msg):
    rospy.loginfo("I heard my name is " + msg.data)

rospy.init_node('receiver') 

sub = rospy.Subscriber('my_topic',String,callback) 

rospy.spin() 
