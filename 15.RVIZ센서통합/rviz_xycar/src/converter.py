#!/usr/bin/env python
import rospy
import time
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

a = -3.14
b = -3.14

def callback(mt_data):
    global a
    global b
    wheel_msg = JointState()
    wheel_msg.header = Header()
    wheel_msg.name = ['front_right_hinge_joint', 'front_left_hinge_joint', 
                        'front_right_wheel_joint', 'front_left_wheel_joint',
                        'rear_right_wheel_joint','rear_left_wheel_joint']
    wheel_msg.velocity = [mt_data.speed]
    wheel_msg.effort = []
    wheel_msg.header.stamp = rospy.Time.now()

    if a >= 3.14:
        a = -3.14
        b = -3.14
    else:
        # 0.01 라디안은 약 6도
        a += 0.005 * mt_data.speed
        b += 0.005 * mt_data.speed

    # front_*_wheel_joint 만 회전 시키기 위한 것
    wheel_msg.position = [mt_data.angle / 50, mt_data.angle / 50, a, b, 0, 0]
    pub.publish(wheel_msg)

motor_control = xycar_motor()
rospy.init_node('converter')
sub = rospy.Subscriber('xycar_motor', xycar_motor, callback)
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
rospy.spin()
