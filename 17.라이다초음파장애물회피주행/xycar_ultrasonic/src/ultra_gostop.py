#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import time
from std_msgs.msg import Int32MultiArray
from xycar_motor.msg import xycar_motor

ultra_msg = None
motor_msg = xycar_motor()
def ultra_callback(data):
    global ultra_msg
    # 초음파센서 토픽이 들어오면 실행되는 콜백함수 정의
    ultra_msg = data.data

def drive_go():
    global motor_msg
    motor_msg.speed = 20
    motor_msg.angle = 0
    pub.publish(motor_msg)

def drive_stop():
    global motor_msg
    motor_msg.speed = 0
    motor_msg.angle = 0 
    pub.publish(motor_msg)


rospy.init_node("ultra_node")
rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

while not rospy.is_shutdown():
    # 초음파센서 토픽이 오면 콜백함수가 호출되도록 세팅
    if ultra_msg == None:
        continue
    
    if (min(ultra_msg[1:4]) <= 40) :
        drive_stop()
    else:
        # 멈추지 않았을 때 계속 전진
        drive_go()
