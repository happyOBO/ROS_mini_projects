#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy, time
from std_msgs.msg import Int32MultiArray
from xycar_motor.msg import xycar_motor

rospy.init_node("ultra_node")

motor_msg = xycar_motor()
ultra_msg = None
rate = rospy.Rate(10)

def ultra_callback(data):
    global ultra_msg
    # 초음파센서 토픽이 들어오면 실행되는 콜백함수 정의
    ultra_msg = data.data



def drive_go():
    global motor_msg
    motor_msg.speed = 30
    motor_msg.angle = 0
    pub.publish(motor_msg)
    rate.sleep()

def drive_opposite(angle):
    global motor_msg
    
    motor_msg.angle = 0
    for sp in range(40):
        motor_msg.speed = 10 - sp
        pub.publish(motor_msg)
        rate.sleep()

    # 반대방향 주행
    motor_msg.angle = 40 * (-angle) if (angle != 0) else 40
    for sp in range(50):
        motor_msg.speed = -5 + min(sp ,35)
        pub.publish(motor_msg)
        rate.sleep()
 
rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)



time.sleep(3) #ready to connect lidar 

while not rospy.is_shutdown():
    # 초음파센서 토픽이 오면 콜백함수가 호출되도록 세팅
    if ultra_msg == None:
        continue
    for i in range(1,4):
        if(ultra_msg[i] <= 25):
            drive_opposite(i-2)
            break
    else:
        # 멈추지 않았을 때 계속 전진
        drive_go()





