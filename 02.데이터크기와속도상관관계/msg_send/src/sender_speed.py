#!/usr/bin/env python

import rospy 
from std_msgs.msg import String 

rospy.init_node('sender') 

pub = rospy.Publisher('long_string',String) 

rate = rospy.Rate(2) 

size = 50000000
while not rospy.is_shutdown(): 
    pub.publish('#' * size) 
    rate.sleep() 

