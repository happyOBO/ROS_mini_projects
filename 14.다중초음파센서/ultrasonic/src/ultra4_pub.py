#!/usr/bin/env python2

import serial, time, rospy
from std_msgs.msg import Int32MultiArray

msg = Int32MultiArray()

ser_front = serial.Serial( \
    port='/dev/ttyUSB0', \
    baudrate=9600,
)

def read_sensor():
    global msg
    serial_data = ''
    serial_data = ser_front.readline()
    ultrasonic_list = serial_data.split("mm")
    msg.data = list(map(lambda x: int(x), ultrasonic_list[:-1]))

if __name__ == '__main__':
    rospy.init_node('ultrasonic4_pub', anonymous=False)
    pub = rospy.Publisher('ultra4', Int32MultiArray, queue_size=1)
    rate = rospy.Rate(10)
    while(not rospy.is_shutdown()):
        read_sensor()
        pub.publish(msg)
        rate.sleep()
    ser_front.close()


