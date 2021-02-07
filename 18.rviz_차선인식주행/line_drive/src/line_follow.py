#!/usr/bin/env python

import cv2, time
import rospy
import numpy as np
from xycar_motor.msg import xycar_motor
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
cv_image = np.empty(shape=[0])
motor_msg = xycar_motor()

def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

rospy.init_node('driver')
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

def drive(left,right):
    global width_640, motor_msg ,motor_pub
    motor_msg.header = Header()
    motor_msg.header.stamp = rospy.Time.now()
    if( left >= 0 and right > 0):
        diff = width_640 - (left + right)
        motor_msg.speed = 10
        motor_msg.angle = diff * 0.4
    motor_pub.publish(motor_msg)

threshold_100 = 100

width_640 = 640
scan_width_200, scan_height_20 = 200, 20
lmid_200, rmid_440 = scan_width_200, width_640 - scan_width_200
area_width_20, area_height_10 = 20, 10
vertical_430 = 430
row_begin_5 = (scan_height_20 - area_height_10) // 2
row_end_15 = row_begin_5 + area_height_10
pixel_threshold_160 = 0.8 * area_width_20 * area_height_10

while not rospy.is_shutdown():
    if cv_image.size != (640*480*3):
        continue
    roi = cv_image[vertical_430:vertical_430 + scan_height_20, :]
    cv_image = cv2.rectangle(cv_image, (0, vertical_430),
        (width_640 - 1, vertical_430 + scan_height_20),
        (255, 0, 0), 3)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lbound = np.array([0, 0, threshold_100], dtype=np.uint8)
    ubound = np.array([131, 255, 255], dtype=np.uint8)

    bin = cv2.inRange(hsv, lbound, ubound)
    view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

    left, right = -1, -1

    for l in range(area_width_20, lmid_200):
        area = bin[row_begin_5:row_end_15, l - area_width_20:l] 
        if cv2.countNonZero(area) > pixel_threshold_160:
            left = l
            break

    for r in range(width_640 - area_width_20, rmid_440, -1):
        area = bin[row_begin_5:row_end_15, r:r + area_width_20]
        if cv2.countNonZero(area) > pixel_threshold_160:
            right = r
            break

    if left != -1:
        lsquare = cv2.rectangle(view,
                                (left - area_width_20, row_begin_5),
                                (left, row_end_15),
                                (0, 255, 0), 3)
    else:
        print("Lost left line")

    if right != -1:
        rsquare = cv2.rectangle(view,
                                (right, row_begin_5),
                                (right + area_width_20, row_end_15),
                                (0, 255, 0), 3)
    else:
        print("Lost right line")

    if(left != -1 and right != -1):
        drive(left,right)

    cv2.imshow("origin", cv_image)
    cv2.imshow("view", view)

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lbound = np.array([0, 0, threshold_100], dtype=np.uint8)
    ubound = np.array([131, 255, 255], dtype=np.uint8)   
    hsv = cv2.inRange(hsv, lbound, ubound)
    cv2.imshow("hsv", hsv) 

    cv2.waitKey(1)


cap.release()
cv2.destroyAllWindows()

