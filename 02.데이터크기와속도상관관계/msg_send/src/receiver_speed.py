#!/usr/bin/env python


import rospy 
from std_msgs.msg import String 
import time

prev_time = time.time()

size = 50000000 # 데이터 크기
def callback(msg):
    global prev_time, size
    curr_time = time.time()
    # 속도 및 데이터 크기, 걸린 시간 출력
    rospy.loginfo("%d bytes %f seconds \n speed : %f byte/s" \
    %(size , (curr_time - prev_time), size / (curr_time - prev_time)))
    prev_time = curr_time

rospy.init_node('receiver') 

sub = rospy.Subscriber('long_string',String,callback) 

rospy.spin() 
