#!/usr/bin/env python2

import serial, time, rospy
from std_msgs.msg import Int32

msg = Int32()

ser_front = serial.Serial( \
    port='/dev/ttyUSB0', \
    baudrate=9600,
)

def read_sensor():
    global msg
    serial_data = ''
    # while ser_front.inWaiting():
    #     s = ser_front.read()
    #     if(s != '\n'):
    #         serial_data += s
    #     else:
    #         break
    serial_data = ser_front.readline()
    ultrasonic_data = int(filter(str.isdigit, serial_data))
    msg.data = ultrasonic_data

if __name__ == '__main__':
    rospy.init_node('ultrasonic_pub', anonymous=False)
    pub = rospy.Publisher('ultrasonic', Int32, queue_size=1)
    rate = rospy.Rate(10)
    while(not rospy.is_shutdown()):
        read_sensor()
        pub.publish(msg)
        rate.sleep()
    ser_front.close()