#!/usr/bin/env python
import rospy
import serial
import math
import sys

from sensor_msgs.msg import Imu
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion, Vector3, Pose
from tf.transformations import quaternion_from_euler
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from visualization_msgs.msg import Marker

def show_text_in_rviz(mark_pub, markeroutput_txt):
    marker = Marker(text=output_txt)
    mark_pub.publish(marker)

rospy.init_node("razor_node")

pub_mode = rospy.get_param('~rviz_mode', 'true')
if pub_mode == "true":
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)

default_port='/dev/ttyIMU'
port = rospy.get_param('~port', default_port)

rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=57600, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+ port + ". Did you specify the correct port in the launch file?")
    sys.exit(0)

pub = rospy.Publisher('imu', Imu, queue_size=1)
diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
diag_pub_time = rospy.get_time();

imuMsg = Imu()
imuMsg.header.frame_id = 'map'

diag_arr = DiagnosticArray()

diag_msg = DiagnosticStatus()
diag_msg.name = 'Razor_Imu'
diag_msg.message = 'Received AHRS measurement'

marker = Marker(color=ColorRGBA(r=1, g=1, b=1, a=1), type=9, id=0, scale=Vector3(x=0, y=0, z=0.14), pose=Pose(position=Vector3(x=0.5, y=0.5, z=1.45), orientation=Quaternion(w=1, x=0, y=0, z=0)), ns="imu")

roll=0
pitch=0
yaw=0
seq=0

degrees2rad = math.pi/180.0

rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")
rospy.sleep(5) # Sleep for 5 seconds to wait for the board to boot

rospy.loginfo("Flushing first 200 IMU entries...")
for x in range(0, 200):
    line = ser.readline()

rospy.loginfo("Publishing IMU data...")
while not rospy.is_shutdown():
    line = ser.readline()
    line = line.replace("#YPR=","")   # Delete "#YPRAG="

    words = str(line).split(",")    # Fields split

    if len(words) > 2:
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
        yaw_deg = float(words[0])
        if yaw_deg > 180.0:
            yaw_deg = yaw_deg - 360.0
        if yaw_deg < -180.0:
            yaw_deg = yaw_deg + 360.0
        yaw = yaw_deg*degrees2rad

        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        pitch_deg = float(words[1])
        if pitch_deg > 180.0:
            pitch_deg = pitch_deg - 360.0
        if pitch_deg < -180.0:
            pitch_deg = pitch_deg + 360.0
        pitch = pitch_deg*degrees2rad

        roll_deg = float(words[2])
        if roll_deg > 180.0:
            roll_deg = roll_deg - 360.0
        if roll_deg < -180.0:
            roll_deg = roll_deg + 360.0
        roll = roll_deg*degrees2rad

    if pub_mode == "true":
        output_txt = "roll : "+str(round(roll,2))+", "
        output_txt += "pitch : "+str(round(pitch,2))+", "
        output_txt += "yaw : "+str(round(yaw,2))

        marker.header = Header(stamp=rospy.Time.now(), frame_id="map")
        marker.text = output_txt

        marker_pub.publish(marker)

    q = quaternion_from_euler(roll,pitch,yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]

    imuMsg.header.stamp= rospy.Time.now()

    imuMsg.header.seq = seq
    seq = seq + 1
    pub.publish(imuMsg)

    if (diag_pub_time < rospy.get_time()) :
        diag_pub_time += 1
        
        diag_arr.header.stamp = rospy.get_rostime()
        diag_arr.header.frame_id = '1'
        
        diag_msg.level = DiagnosticStatus.OK

        diag_msg.values.append(KeyValue('roll (deg)', str(roll*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('pitch (deg)', str(pitch*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('yaw (deg)', str(yaw*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('sequence number', str(seq)))
        diag_arr.status.append(diag_msg)
        diag_pub.publish(diag_arr)
        
ser.close

