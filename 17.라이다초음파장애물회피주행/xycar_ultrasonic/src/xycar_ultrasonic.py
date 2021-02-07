#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : cam_tune.py
# 작 성 자 : 자이트론
# 생 성 일 : 2020년 07월 23일
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import serial, time ,rospy
from std_msgs.msg import Int32MultiArray

import signal
import sys
import os

def signal_handler(sig, frame):
    time.sleep(2)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class ultrasonic_pub:

   ser_back = serial.Serial(
       port='/dev/ttySonicBACK',
       baudrate=38400,
   )
   ser_front = serial.Serial(
       port='/dev/ttySonicFRONT',
       baudrate=38400,
   )

   FRONT = [0,0,0,0,0]
   BACK = [0,0,0,0,0]

   def __init__(self):
      self.pub = rospy.Publisher('xycar_ultrasonic', Int32MultiArray, queue_size=1)
      self.ultra_sonic = Int32MultiArray()

   def send(self):
      num_B = 0
      Value_B = 0
      num_F = 0
      Value_F = 0

      self.FRONT[0] = 0
      self.BACK[0] = 0

      for i in range(0,4):
          num_B, Value_B = self.read_value(self.ser_back.readline())
          num_F, Value_F = self.read_value(self.ser_front.readline())
          self.BACK[num_B] = Value_B
          self.FRONT[num_F] = Value_F
          
      self.ser_back.flushInput()
      self.ser_front.flushInput()

      self.ultra_sonic.data = [self.FRONT[1], self.FRONT[2], self.FRONT[3], self.FRONT[4], self.BACK[1], self.BACK[2], self.BACK[3], self.BACK[4]]
      #print(self.ultra_sonic.data)
      self.pub.publish(self.ultra_sonic)


   def read_value(self, serial_value):
      stri = str(serial_value[:-2])
      if len(stri) == len(filter(str.isalpha, stri)):
         return 0, -1

      int_serial_value = int(filter(str.isdigit, stri))

      value_number = int_serial_value % 10
      value = int_serial_value // 10

      if (value_number > 4) or (value_number < 0):
         return 0, -2

      return value_number, value

if __name__ == '__main__':

   rospy.init_node('xycar_ultrasonic', anonymous=False)
   ultrasonic = ultrasonic_pub()   
     
   while not rospy.is_shutdown():
      ultrasonic.send()
   try:
      rospy.spin()
   except KeyboardInterrupt:
      print("Shutting down")
