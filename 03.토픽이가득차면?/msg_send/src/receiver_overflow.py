#!/usr/bin/env python

import rospy 
from std_msgs.msg import Int32

prev_num = 1
def callback(msg):
    for i in range(1000000):
        hello = 0
    rospy.loginfo(msg.data)

    

rospy.init_node('receiver') 

sub = rospy.Subscriber('my_topic',Int32,callback,queue_size = 3) 

rospy.spin() 