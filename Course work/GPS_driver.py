#!/usr/bin/env python

import rospy
import serial
import utm
import rosbag
import time
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int32
from lab3.msg import GPGGA

if __name__ == '__main__':
	rospy.init_node('gps_location')
	ser = serial.Serial('/dev/ttyACM0', baudrate=57600)
	gps_pub = rospy.Publisher('location',GPGGA, queue_size=10)

	while not rospy.is_shutdown():
		
		try:

			line = ser.readline()	

			if line.startswith( '$GNGGA' ) :
				
				line = line.split(',')
				print line

				#print line
				
				e_w =  str(line[5])
				
				InputData = ((line[0]),float(line[2]),float(line[4]),float(line[9]))
				#print InputData
				
				lat = InputData[1]
				lat = str(lat)
				lat_deg = int(lat[:2])
				lat_min = float(lat[2:])
				lat_min = lat_min/60
				lat = lat_deg + lat_min
				lat = float(lat)
				qual = int(line[6])
				hdop = float(line[8])
				sat = int(line[7])
				#print lat
				
				lon = InputData[2]
				lon = str(lon)
				lon_deg = int(lon[:2])
				lon_min = float(lon[2:])
				lon_min = lon_min/60
				lon = lon_deg + lon_min
				
				if e_w == 'W':
					lon = float(lon*(-1))
				#print lon

				alt = InputData[3]
				#print alt

				new_coord = utm.from_latlon(lat, lon)
				print new_coord
				east = new_coord[0]
				north = new_coord[1]
				zone = new_coord[2]
				letter = new_coord[3]
				# print east
				# print north
				# print zone
				# print letter

				msg = GPGGA()
				msg.lat = lat
				msg.lon = lon
				msg.alt = alt
				msg.east = east
				msg.north = north
				msg.letter = letter
				msg.zone = zone
				msg.qual = qual
				msg.hdop = hdop
				msg.sat = sat

				rospy.loginfo(msg)
				gps_pub.publish(msg)
				
				

		except rospy.ROSInterruptException:
			pass
