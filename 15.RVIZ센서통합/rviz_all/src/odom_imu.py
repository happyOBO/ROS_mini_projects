#!/usr/bin/env python
#-*- coding:utf-8 -*-

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState , Imu



curr_wheel_arrow = None
wheel_angle = None

imu_euler = None



def callback_motor(joint_data):
    global curr_wheel_arrow, wheel_angle

    curr_wheel_arrow = joint_data.position[2]
    wheel_angle = joint_data.position[0]

    
def callback_imu(imu_data):
    global imu_euler
    euler =  tf.transformations.euler_from_quaternion((imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w))
    imu_euler = [euler[0],euler[1],euler[2]] 



rospy.init_node('rviz_odom')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
rospy.Subscriber('joint_states', JointState, callback_motor)
rospy.Subscriber('imu', Imu, callback_imu)
odom_broadcaster = tf.TransformBroadcaster()
last_time = rospy.Time.now()
rate = rospy.Rate(10)


x = 0.0
y = 0.0
th = 0.0
r = 0.0
p = 0.0
last_wheel_arrow = 0.0

while not rospy.is_shutdown():
    if(curr_wheel_arrow == None or imu_euler == None):
        continue

    current_time = rospy.Time.now()

    # when last_wheel_arrow >= 3.14 and curr_wheel_arrow == -3.14 , delta_ang is sum of them.
    delta_wheel_arrow =  curr_wheel_arrow + last_wheel_arrow if (curr_wheel_arrow * last_wheel_arrow < 0)  else curr_wheel_arrow - last_wheel_arrow

    # distance = 2 * delta_angle * wheel's radius
    vx = 2 * delta_wheel_arrow * 1.8
    vy = 2 * delta_wheel_arrow * 1.8

    dt = (current_time - last_time).to_sec()
    delta_x = vx * cos(th) * dt
    delta_y = vy * sin(th) * dt
    delta_th = ( wheel_angle) * dt 

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    # oiler value -> quaternion value
    odom_quat = tf.transformations.quaternion_from_euler(imu_euler[0] , imu_euler[1], th + imu_euler[2])

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, wheel_angle * dt))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    last_wheel_arrow = curr_wheel_arrow

    rate.sleep()
    
    
