#!/usr/bin/env python

import serial, time, rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Header

rospy.init_node('lidar_range')

pub = [None,None,None,None]

# 4개의 토픽 발행
for i in range(4):
    name = 'scan'+str(i)
    pub[i] = rospy.Publisher(name, Range, queue_size=1)


msg = Range()
h = Header()
h.frame_id = "sensorXY"
msg.header = h
msg.radiation_type = Range().ULTRASOUND
msg.min_range = 0.02 # 2cm
msg.max_range = 2.0 # 2m
msg.field_of_view = (30.0/180.0) * 3.14


while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()

    # msg.range에 장애물까지의 거리를 미터 단위로 넣고 토픽을 발행한다.
    msg.range = 0.4

    for i in range(4):
        pub[i].publish(msg)
        msg.range += 0.4

    time.sleep(0.2)



