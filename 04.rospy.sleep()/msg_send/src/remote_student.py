#!/usr/bin/env python

import rospy
from msg_send.msg import my_msg
from std_msgs.msg import String


def callback(msg):
    print(msg.data)

rospy.init_node('msg_sender', anonymous=True)
pub = rospy.Publisher('msg_to_xycar',my_msg)

msg = my_msg()
msg.first_name= "Jeongmin"
msg.last_name = "Jo"
msg.age = 23
msg.score =100
msg.id_number=13
msg.phone_number="010-1234-5678"

sub = rospy.Subscriber('msg_from_xycar',String,callback) 


rate = rospy.Rate(1)
while not rospy.is_shutdown():
    pub.publish(msg)
    print("sending message")
    rate.sleep()