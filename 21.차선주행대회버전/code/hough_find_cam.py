#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge


######## 고정값 ########

# 영상 크기
Width, Height  = 640, 480

# 색상 코드
red, green, blue, yellow = (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)

# 이미지 초기값
img_from_cam = np.array([])




######## 중앙선 영상처리 설정값 ########

# ROI 영역 설정
m_offset_y, m_gap_y, m_offset_x, m_gap_x = 280, 50, 290, 60

# HoughLineP 설정
hough_threshold_m, min_length_m, min_gap_m = 5, 15, 20

# 차선 기울기 필터링 범위
slope_threshold = 0.5




######## 양쪽 차선 영상처리 설정값 ########

# ROI 영역 설정
Offset, Gap = 340, 40

# HoughLineP 설정
hough_threshold, min_length, min_gap = 30, 30, 10

# 차선 기울기 필터링 범위
low_slope_threshold , high_slope_threshold = 0, 2

# 왼쪽 오른쪽 구별 설정
divide_margin = 120

# gaussianBlur 설정
kernel_size = (5,5)

# Canny 설정
low_threshold, high_threshold = 60, 70



######## 함수 ########

# 콜백 함수
def img_callback(data):
    global img_from_cam
    img_from_cam = bridge.imgmsg_to_cv2(data, "bgr8")

# 직선 그리기 함수
def draw_lines(frame, x1, y1, x2, y2, offset_x, offset_y, color=None):
        if color is None:
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        cv2.line(frame, (x1+offset_x, y1+offset_y), (x2+offset_x, y2+offset_y), color, 2)

# 직사각형 그리기 함수
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2
    cv2.rectangle(img, (lpos - 5, 15 + offset), (lpos + 5, 25 + offset), green, 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset), (rpos + 5, 25 + offset), green, 2)
    cv2.rectangle(img, (center-5, 15 + offset), (center+5, 25 + offset), green, 2)    
    cv2.rectangle(img, (315, 15 + offset), (325, 25 + offset), red, 2)
    cv2.rectangle(img, (0, Offset), (Width - 1, Offset + Gap), blue, 3)
    cv2.rectangle(img, (m_offset_x, m_offset_y), (m_offset_x+m_gap_x, m_offset_y+m_gap_y), yellow, 2)

# 양쪽 차선 구별 함수
def divide_left_right(frame, lines):
    left, right = [], []
    for x1, y1, x2, y2 in lines[:, 0]:
        m = float("inf") if x1 == x2 else float(y2-y1) / float(x2-x1)

        # 기울기 필터링
        if not low_slope_threshold < abs(m) < high_slope_threshold:
            continue
        b = y1 - m * x1

        # 기울기가 음수이고 왼쪽에 있으면 왼쪽 차선
        if m < 0 and x2 < Width/2 - divide_margin:
            left.append([m, b])
            draw_lines(frame, x1, y1, x2, y2, 0, Offset)

        # 기울기가 양수이고 오른쪽에 있으면 왼쪽 차선   
        elif m > 0 and x1 > Width/2 + divide_margin:
            right.append([m, b])
            draw_lines(frame, x1, y1, x2, y2, 0, Offset)

    return left, right

# 대표값 추출 함수
def get_line_pos(line, left=True):
    
    # 차선을 찾지 못한 경우 끝 지점으로 설정
    x1 = x2 = pos = 0 if left else Width

    # 차선을 찾은 경우 여려 직선중 중앙값으로
    if line:
        m, b = np.median(line, 0)
        pos = ((Gap / 2) - b) / m
        b += Offset
        x1, x2 = (Height - b) / float(m), ((Height/2) - b) / float(m)

    cv2.line(image, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)
    return int(pos)

# 양쪽 차선 영상처리 과정
def process_roi(frame):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, kernel_size, 0)
    canny = cv2.Canny(blur, low_threshold, high_threshold)
    roi = canny[Offset : Offset+Gap, 0 : Width]
    #cv2.imshow("roi", roi)

    lines = cv2.HoughLinesP(roi, 1, math.pi/180, hough_threshold, min_length, min_gap)
    error = 0
    if lines is not None:
        left_line, right_line = divide_left_right(frame, lines)
        lpos, rpos = get_line_pos(left_line, left=True), get_line_pos(right_line, left=False)
        draw_rectangle(frame, lpos, rpos, offset=Offset)
        center = (lpos + rpos) / 2
        error = center-320
    return error

# 중앙선 영상처리 과정
def process_m_roi(img):
    roi = img[m_offset_y : m_offset_y + m_gap_y, m_offset_x : m_offset_x + m_gap_x]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
    lines = cv2.HoughLinesP(binary, 1, math.pi/180, hough_threshold_m, None, min_length_m, min_gap_m)
    is_straight = 0
    if lines is not None:
        is_straight =  get_state(img, lines)
        
    # cv2.imshow("binary", binary)
    # cv2.imshow('curve', roi_curve)

    return is_straight

# 직선/커브 판별 함수
def get_state(img, lines):
    is_straight = False
    for x1, y1, x2, y2 in lines[:, 0]:
        m = float("inf") if y1 == y2 else float(x2-x1) / float(y2-y1)
        if abs(m) < slope_threshold :
            is_straight = True
            cv2.line(img, (x1+m_offset_x, y1+m_offset_y), (x2+m_offset_x, y2+m_offset_y), (0,255,255), 2)
    return is_straight




######## main ########

if __name__ == '__main__':

    rospy.init_node("line_find")
    bridge = CvBridge()

    pub = rospy.Publisher("error", Int32MultiArray, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

    msg = Int32MultiArray()
    msg.data = [0,1]

    while not rospy.is_shutdown():
        if img_from_cam.shape != (Height, Width ,3):
            continue
        
        image = img_from_cam.copy()

        # 중앙선 영상 처리
        is_straight = process_m_roi(image)

        # 양쪽 차선 영상 처리
        error = process_roi(image)      

        # 결과값 발행
        msg.data = [error,is_straight]
        pub.publish(msg)

        cv2.imshow("cam", image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

