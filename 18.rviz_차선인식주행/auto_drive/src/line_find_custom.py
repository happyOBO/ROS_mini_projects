#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node("line_find")

bridge = CvBridge()
img = np.empty(shape=[0])

def img_callback(data):
    global img
    img = bridge.imgmsg_to_cv2(data, "bgr8")

pub = rospy.Publisher("left_right", Int32MultiArray, queue_size=1)
rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
rate = rospy.Rate(30)

threshold_60 = 120

width_640 = 640
scan_width_200, scan_height_20 = 200, 20
lmid_200, rmid_440 = scan_width_200, width_640 - scan_width_200
area_width_20, area_height_10 = 20, 10
vertical_430 = 330
row_begin_5 = (scan_height_20 - area_height_10) // 2
row_end_15 = row_begin_5 + area_height_10
pixel_threshold_160 = 0.4 * area_width_20 * area_height_10

msg = Int32MultiArray()
msg.data = [0,0]

while not rospy.is_shutdown():
    frame = img.copy()
    if frame.size != (640*480*3):
        continue
    roi = frame[vertical_430:vertical_430 + scan_height_20, :]
    frame = cv2.rectangle(frame, (0, vertical_430),
        (width_640 - 1, vertical_430 + scan_height_20),
        (255, 0, 0), 3)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lbound = np.array([0, 0, threshold_60], dtype=np.uint8)
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

    cv2.imshow("origin", frame)
    cv2.imshow("view", view)
    if cv2.waitKey(1) & 0xFF == 27:
        break
    
    # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # lbound = np.array([0, 0, threshold_60], dtype=np.uint8)
    # ubound = np.array([131, 255, 255], dtype=np.uint8)   
    # hsv = cv2.inRange(hsv, lbound, ubound)
    # cv2.imshow("hsv", hsv) 
    msg.data = [left,right]
    pub.publish(msg)
    rate.sleep()

cv2.destroyAllWindows()

