#!/usr/bin/env python
import rospy
import time
from xycar_motor.msg import xycar_motor

motor_control = xycar_motor()
rospy.init_node('auto_driver')
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

def motor_pub(angle, speed):
    global pub
    global motor_control
    motor_control.angle = angle
    motor_control.speed = speed
    pub.publish(motor_control)

speed = 3
turn_angle = 20
straight_angle = 0
rate = rospy.Rate(50)

while not rospy.is_shutdown():
    # turn_angle : 좌회전 또는 우회전 각도
    # straight_angle : 직진 각도
    # 좌회전 또는 우회전
    for _ in range(0,700):
        motor_pub(turn_angle,speed)
        rate.sleep()
    for _ in range(0,200):
        motor_pub(straight_angle,speed)
        rate.sleep()
    # 좌회전 방향이면 우회전으로, 우회전 방향이면 좌회전 각도로 변경
    turn_angle *= -1

    

