#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Bool

def callback(ack):
    if(ack.data):
        pub.publish('first')
        send_ack.publish(True)
        rate.sleep()


rospy.init_node('first')
pub = rospy.Publisher('my_topic', String, queue_size=0)
send_ack = rospy.Publisher('first_ack', Bool, queue_size=0)
receive_ack = rospy.Subscriber('second_ack',Bool,callback)

rate = rospy.Rate(1)
for i in range(5):
    pub.publish('first')
    rospy.loginfo("hello")
    send_ack.publish(True)
    rate.sleep()

rospy.spin()

