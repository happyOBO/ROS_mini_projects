#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from xycar_motor.msg import xycar_motor
from std_msgs.msg import Header
from std_msgs.msg import Int32

class MovingAverage:
    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data = self.data[1:] + [new_sample]

    def get_mm(self):
        return float(sum(self.data)) / len(self.data)

    def get_wmm(self):
        s = 0
        for i, x in enumerate(self.data):
            s += x * self.weights[i]
        return float(s) / sum(self.weights[:len(self.data)])


def drive(angle):
    global pub
    motor_msg = xycar_motor()
    motor_msg.header = Header()
    motor_msg.header.stamp = rospy.Time.now()
    motor_msg.speed = 10
    motor_msg.angle = - angle * 0.4
    pub.publish(motor_msg)

angle_filter = MovingAverage(10)

def callback(data):
    angle_filter.add_sample(data.data)
    avg_angle = angle_filter.get_mm()
    drive(avg_angle)

if __name__ == '__main__':
    rospy.init_node('driver')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber('wheel_angle', Int32, callback)
    rospy.spin()