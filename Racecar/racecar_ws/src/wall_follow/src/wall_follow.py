#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

kp = 10
kd = .1
#ki = 
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

ANGLE_RANGE = 180 # Laserscan from ZED data -pi/2 to pi/2
DESIRED_DISTANCE_RIGHT = 0.5 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # meters



def lidar_callback(data):
	
	Index1 = 110
	Index2 = 170
	ang_rad_1 = Index1 * 2 * 3.14159265 / 360
	ang_rad_2 = Index2 * 2 * 3.14159265 / 360

	a = getRange(data, Index2)
	print (a)
	b = getRange(data, Index1)
	print (b)


	alpha = math.atan((a*math.cos(ang_rad_2 - ang_rad_1) - b) / (a * math.sin(ang_rad_2 - ang_rad_1)))

	AB = b * math.cos(alpha)
	
	AC = VELOCITY * 0.1 # 1 sec time step t+1

	rightDist = AB + AC * math.sin(alpha)

	error = followRight(data, rightDist)
	
        #send error to pid_control
        pid_control(error, VELOCITY)
	print(VELOCITY)

def getRange(data, index):
        # data: single message from topic /camera/scan
        # Outputs length in meters to object with angle in lidar scan field of view
        
	
	dist = data.ranges[index]
	return dist
	

def pid_control(error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0.0
        # Use kp, ki & kd to implement a PID controller 

	angle = -(kp * error + kd * (prev_error - error))
	prev_error = error


        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        drive_pub.publish(drive_msg)

def followRight(data, rightDist):
	global DESIRED_DISTANCE_RIGHT
	error = DESIRED_DISTANCE_RIGHT - rightDist

        return error

# Publish to drive
# drive_pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
# drive_pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=10) 
drive_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/output', AckermannDriveStamped, queue_size=10)


class WallFollow:

    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/camera/scan'
        #drive_topic = '/drive'

        self.lidar_sub = rospy.Subscriber('/camera/scan', LaserScan, lidar_callback)  # Subscribe to LIDAR
        #drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10) # Publish to drive
	





def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    #rospy.Subscriber('/camera/scan', LaserScan, getRange)  # Subscribe to LIDAR
    #rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10) # Publish to drive
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()


if __name__=='__main__':
	main(sys.argv)
