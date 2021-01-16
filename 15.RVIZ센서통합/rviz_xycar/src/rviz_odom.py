#!/usr/bin/env python2

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState


def callback(joint_data):
    global x, y, th, last_whl_ang, last_time
    current_time = rospy.Time.now()
    curr_whl_ang = joint_data.position[2]

    # when last_whl_ang >= 3.14 and curr_whl_ang == -3.14 , delta_ang is sum of them.
    delta_whl_ang =  curr_whl_ang + last_whl_ang if (curr_whl_ang * last_whl_ang < 0)  else curr_whl_ang - last_whl_ang

    # distance = 2 * delta_angle * wheel's radius
    vx = 2 * delta_whl_ang * 1.8
    vy = 2 * delta_whl_ang * 1.8

    dt = (current_time - last_time).to_sec()
    delta_x = vx * cos(th) * dt
    delta_y = vy * sin(th) * dt
    delta_th = joint_data.position[0] * dt 

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    # oiler value -> quaternion value
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

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
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, joint_data.velocity[0] * dt))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    last_whl_ang = curr_whl_ang



rospy.init_node('rviz_odom')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
sub = rospy.Subscriber('joint_states', JointState, callback)
odom_broadcaster = tf.TransformBroadcaster()


x = 0.0
y = 0.0
th = 0.0
last_whl_ang = 0.0
last_time = rospy.Time.now()


rospy.spin()



