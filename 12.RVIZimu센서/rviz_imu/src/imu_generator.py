#!/usr/bin/env python

import rospy, math, os
from sensor_msgs.msg import Imu
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node('imu_generator', anonymous=False)
pub = rospy.Publisher("imu", Imu, queue_size=1)

rate = rospy.Rate(10)

f = open(os.path.dirname(__file__)+"/imu_data.txt", 'r')
imu_datas = f.readlines()
broadcaster = tf.TransformBroadcaster()
for imu_dt in imu_datas:
    value_list = imu_dt.split(" ")
    quat = quaternion_from_euler(float(value_list[2][:-1]), \
                                float(value_list[5][:-1]), \
                                float(value_list[8][:-1]))
    imu_data = Imu()
    # rospy.loginfo(quat)
    
    broadcaster.sendTransform(
        (0, 0, 0.),
        quat,
        rospy.Time.now(),
        "map",
        "imu"
    )

    imu_data.header.stamp = rospy.Time.now()
    imu_data.header.frame_id = "imu"
    imu_data.orientation.x = quat[0]
    imu_data.orientation.y = quat[1]
    imu_data.orientation.z = quat[2]
    imu_data.orientation.w = quat[3]
    pub.publish(imu_data)
    rate.sleep()
f.close()

