#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
from xycar_motor.msg import xycar_motor

rospy.init_node('driver')

steering_rate2 = 0.5
steering_rate = 0.5
params = [0, 0] # [angle, speed]
rate = rospy.Rate(30)
speed = 30
def callback(msg):
    if msg.data==[-1,-1]:
    	print("sdfds")
    elif msg.data[0] == -1 :
        params[0] = -1 * (640 - msg.data[1]) * steering_rate2
        params[1] = speed
    else:
        params[0] = max(0, msg.data[0]-30) * steering_rate2
        params[1] = speed

def motor_pub():
    motor_control.angle = params[0]
    motor_control.speed = params[1]
    pub.publish(motor_control)

pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
sub = rospy.Subscriber('left_right', Int32MultiArray, callback, queue_size=1)

motor_control = xycar_motor()
while not rospy.is_shutdown():
    motor_pub()
    rate.sleep()
