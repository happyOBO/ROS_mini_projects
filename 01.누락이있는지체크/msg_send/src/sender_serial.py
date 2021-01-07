#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32,Bool

count = 1

def callback(ack):
    global count
    if(ack.data):
        # ACK == True 이면 count를 1 증가시킨다.
        count = count +1


rospy.init_node('sender_serial')
pub = rospy.Publisher('my_topic', Int32)
receive_ack = rospy.Subscriber('ack',Bool,callback)
rate = rospy.Rate(2)

while (not rospy.is_shutdown()):
    pub.publish(count)
    print(count)
    rate.sleep()
