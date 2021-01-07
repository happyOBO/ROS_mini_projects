#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Bool

def callback(ack):
    if(ack.data):
        pub.publish('third')
        send_ack.publish(True)
        rate.sleep()


rospy.init_node('third')
pub = rospy.Publisher('my_topic', String, queue_size=0)
send_ack = rospy.Publisher('third_ack', Bool, queue_size=0)
receive_ack = rospy.Subscriber('second_ack',Bool,callback)
rate = rospy.Rate(1)
rospy.spin()
