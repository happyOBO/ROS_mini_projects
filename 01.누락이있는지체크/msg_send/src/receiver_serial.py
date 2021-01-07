#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32,Bool

ack = False
def callback(msg):
    print (msg.data)
    # 데이터를 받았으니 ACK=True 전송
    pub.publish(True)
    

rospy.init_node('receiver_serial')
pub = rospy.Publisher('ack', Bool)
sub = rospy.Subscriber('my_topic',Int32,callback)

while (not rospy.is_shutdown()):
    # 데이터를 받지 않을 때는 ACK=False 전송
    pub.publish(False)

