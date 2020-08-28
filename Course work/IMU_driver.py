#!/usr/bin/env python

# ~Driver for Vectornav VN-100 IMU~

import rospy
import serial
import rosbag

from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int32

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

from geometry_msgs.msg import Vector3

from lab2.msg import RPY

from math import sin, cos
import math
import numpy as np

from tf.transformations import quaternion_from_euler


def talker(): # ~publish variables~
	msg = RPY()
	msg.yaw = yaw
	msg.pitch = pitch
	msg.roll = roll
	msg.magx = magx
	msg.magy = magy
	msg.magz = magz
	msg.accelx = accelx
	msg.accely = accely
	msg.accelz = accelz
	msg.gyrox = gyrox
	msg.gyroy = gyroy
	msg.gyroz = gyroz

	imu_pub.publish(msg)


def vectors(magx, magy, magz, accelx, accely, accelz, gyrox, gyroy, gyroz):
	v = Vector3(gyrox, gyroy, gyroz)  #angular velocity (rad/s)
	a = Vector3(accelx, accely, accelz) #linear acceleration (m/s^2)
	m = Vector3(magx, magy, magz) #magnetic field (G)
	return v, a, m


def pub_quat(x, y, z, w, v, a): #publish IMU messages
	imu_msg = Imu()
	imu_msg.orientation.x = x
	imu_msg.orientation.y = y
	imu_msg.orientation.z = z
	imu_msg.orientation.w = w 
	imu_msg.angular_velocity = v
	imu_msg.linear_acceleration = a
	quat_pub.publish(imu_msg)

def pub_mag(m):
	mag_msg = MagneticField()
	mag_msg.magnetic_field = m
	mag_pub.publish(mag_msg)


if __name__ == '__main__':
	# ~initialize IMU node~
	rospy.init_node('IMU_data')
	
	# ~establish serial interface~
	ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)

	imu_pub = rospy.Publisher('imu_vars',RPY, queue_size=10)
	quat_pub = rospy.Publisher('quaternion',Imu, queue_size=10)
	mag_pub = rospy.Publisher('mag_field',MagneticField, queue_size=10)

	try:
		while not rospy.is_shutdown():

			line = ser.readline()	
			splitline = line.split(',')
			print splitline

			yaw = float(splitline[1])
			print yaw
			pitch = float(splitline[2])
			print pitch
			roll = float(splitline[3])
			print roll
			magx = float(splitline[4])
			print magx
			magy = float(splitline[5])
			print magy
			magz = float(splitline[6])
			print magz
			accelx = float(splitline[7])
			print accelx
			accely = float(splitline[8])
			print accely
			accelz = float(splitline[9])
			print accelz
			gyrox = float(splitline[10])
			print gyrox
			gyroy = float(splitline[11])
			print gyroy
			gyroz = float(splitline[12][:-5])
			print gyroz
			

			v, a, m = vectors(magx, magy, magz, accelx, accely, accelz, gyrox, gyroy, gyroz)
			

			q = quaternion_from_euler(roll, pitch, yaw)
			x, y, z, w = (q[0], q[1], q[2], q[3])
			print x, y, z, w

			vectors(magx, magy, magz, accelx, accely, accelz, gyrox, gyroy, gyroz)
			
			talker()
			pub_quat(x, y, z, w, v, a)
			pub_mag(m)


	except rospy.ROSInterruptException:
		pass
