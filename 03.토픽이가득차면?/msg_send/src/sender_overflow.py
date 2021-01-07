#!/usr/bin/env python

import rospy 
from std_msgs.msg import Int32

rospy.init_node('sender') 

pub = rospy.Publisher('my_topic',Int32) 

rate = rospy.Rate(1000) # 1초에 1000 번씩
count = 1
while not rospy.is_shutdown(): 
    pub.publish(count)
    #print(count)
    count +=1 
    rate.sleep() 