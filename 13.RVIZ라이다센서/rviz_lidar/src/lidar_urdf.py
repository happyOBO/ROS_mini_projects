#!/usr/bin/env python

import serial, time, rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from std_msgs.msg import Header

frame = ["back", "right", "front", "left"]
def lidar_callback(data):
    rg = Range()
    rg.header = Header()
    
    rg.radiation_type = Range().ULTRASOUND
    rg.min_range = 0.00 # 2cm
    rg.max_range = 13.0 # 2m
    for i in range(4):
        rg.header.frame_id = frame[i]
        rg.header.stamp = rospy.Time.now()
        rg.field_of_view = (20.0/180.0) * 3.14
        rg.range = data.ranges[90 * i]
        pub[i].publish(rg)



rospy.init_node('lidar')

rospy.Subscriber('scan', LaserScan, lidar_callback)

pub = [None,None,None,None]

# 4개의 토픽 발행
for i in range(4):
    name = 'scan'+str(i+1)
    pub[i] = rospy.Publisher(name, Range, queue_size=1)

rospy.spin()




