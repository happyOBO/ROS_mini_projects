#!/usr/bin/env python

import rospy 
from std_msgs.msg import Int32
import time

rospy.init_node('teacher') 

pub = rospy.Publisher('my_topic',Int32) 

rate = rospy.Rate(5) 
count = 1
loop = 1000
while not rospy.is_shutdown(): 
    start_time = time.time()
    for i in range(loop):
        pass
    pub.publish(count)
    count +=1 
    done_time = time.time()
    rate.sleep() 
    sleep_time = time.time()
    rospy.loginfo("send : %d total_time : %f execute time : %f " \
    % (count , sleep_time - start_time,  done_time - start_time))
    loop *= 5