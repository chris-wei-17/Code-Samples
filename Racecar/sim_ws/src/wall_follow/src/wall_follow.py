#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

kp = 5
kd = .01
#ki = 
servo_offset = 0.0
prev_error = 0.0 
#error = 0.0
integral = 0.0

ANGLE_RANGE = 360 # Laserscan from ZED data -pi/2 to pi/2, simulated laser scan 360 degrees set in params.yaml 180 beams 2degrees/beam
DESIRED_DISTANCE_RIGHT = .5 # meters
DESIRED_DISTANCE_LEFT = 0.55
#VELOCITY = .500  meters per second
CAR_LENGTH = 0.50 # meters
# dist_fwd = 0.00 # initialized safety distance
class Explore:

    def __init__(self):

        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
	
	# Subscribe to LIDAR
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)  
        # Publish to drive
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10) 
        # publish LIDAR measurement indeces
        #self.measurement_pub = rospy.Publisher('/measurements', LaserScan, queue_size=10)
        
	self.velocity_limit = 1.00
	self.velocity_setpoint = 1.00


    def getRange(self, data, index):

	# data: single message from topic /camera/scan
	# Outputs length in meters to object with angle in lidar scan field of view
	dist = data.ranges[index]
	return dist
	

    def lidar_callback(self, data):
	i = 1
	Index1 = 45
	Index2 = 60 
	center_index = 89
	rear_index = 0
	ang_rad_1 = Index1 * 6.28318 / 180 
	ang_rad_2 = Index2 * 6.28318 / 180  

	a = self.getRange(data, Index2)
	print ("a", a)
	print ("Range Index2", data.ranges[60])

	b = self.getRange(data, Index1)
	print ("b", b)
	print ("Range Index1", data.ranges[45])

	dist_fwd = self.getRange(data, center_index)
	print("Center Index", data.ranges[89])
	print ("dist_fwd", dist_fwd)
	
	r_offset = self.getRange(data, 70)
	l_offset = self.getRange(data, 105)
	
	
	# dist_rear = self.getRange(data, rear_index)
	# print("Rear Index", data.ranges[0])
	# print("dist_rear", dist_rear)
	
	# Measurement messages
	
	#Index_1 = LaserScan()
	#Index_1.header = data.header
	#Index_1.ranges = data.ranges[Index1]

	#Index_2 = LaserScan
	#Index_2.header = data.header
	#Index_2.ranges = data.ranges[Index2]
	
	# Publish measurements for RVIZ
	#self.measurement_pub.publish(Index_1)
	#self.measurement_pub.publish(Index_2)
	
	alpha = math.atan((a*math.cos(ang_rad_2 - ang_rad_1) - b) / (a * math.sin(ang_rad_2 - ang_rad_1)))
	print("alpha", alpha)
	
	if i < 2:
		vel = self.velocity_setpoint
		i = ++i
	else:
		vel = self.pid_control(error, dist_fwd)
		
	AB = b * math.cos(alpha)
	AC = vel * 0.1 # .1 sec time step t+1
	rightDist = AB + AC * math.sin(alpha)
	print("rightDist", rightDist)

	error = self.followRight(data, rightDist)

        #send error to pid_control
        self.pid_control(error, dist_fwd, rightDist, l_offset, r_offset)
	

    def pid_control(self, error, dist_fwd, rightDist, l_offset, r_offset):

	global integral
	global prev_error
	global kp
	global ki
	global kd
	i = 1
	j = 1
	k = 1

	# Avoid getting trapped in corners and circling
	#if ( (rightDist > 2.0) and (dist_fwd > 2.0) ):
		#angle = 0.0
	#elif ( (rightDist > 2.0) and (dist_fwd < 2.0) ):
	#	angle = .5
	#else:
	
	angle = (kp * error + kd * (prev_error - error))
	prev_error = error

	print("Angle", angle)
	
	# Initialize velocity
	if i < 2:
		velocity = self.velocity_setpoint
		i = ++i
	else:
		velocity = self.velocity_setpoint	

	# Speed Control with wall detection and speed limiter
	if ( (dist_fwd < .75) and (dist_fwd > 0.25) ):	
		velocity, angle = self.turn_left(velocity, angle)
	#elif ( (dist_fwd < 0.5) and (dist_fwd > 0.25) ):
		#velocity, angle = self.crawl(velocity, angle)
		#print("TURN")
		#print("TURN VEL", velocity)		
		#velocity = velocity / 2.0
		#angle = .5
	elif dist_fwd < 0.25:
		print("STOP")
		velocity = 0.00
			
	#elif velocity_setpoint > self.velocity_limit:  eventually implement elif for variable speed in corners
		#velocity = self.velocity_limit
	else:
		velocity = self.velocity_setpoint
		print("EVERYTHING IS NORMAL")
	lvar = abs(dist_fwd - l_offset)
	rvar = abs(dist_fwd - r_offset)
#	if ( (abs(error) > 3.0) and (dist_fwd > .75) and (rvar > 1.0) ):
#		velocity, angle = self.turn_right(velocity, angle)
#		print("SEARCHING")
	#elif ( (abs(error) > 3.0) and (dist_fwd > .75) and (lvar > 1.0) ):	
	#	velocity, angle = selt.turn_left(velocity, angle)
	if ( (abs(error) > 3.0) and (dist_fwd > .75) ):
		velocity = 1
		angle = 0
		print("FINDING WALL")
		
	if velocity == 0:
		#j = 1
		while j < 200:
			velocity = -.5
			angle = -.5
			j = j + .1
			print("BACKING UP")
			print(j)
			#if dist_rear < .1:
				#break

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

	print("error", error)

	print("VELOCITY", velocity)
	return velocity


    def followRight(self, data, rightDist):

	global DESIRED_DISTANCE_RIGHT
	error = DESIRED_DISTANCE_RIGHT - rightDist
	if error > .5:
		error = .5
	return error
	print("error", error)
    
    def turn_left(self, velocity, angle):
	print("TURN")
	print("TURN VEL", velocity)	
	velocity = velocity / 2.0
	angle = .5
	return velocity, angle
	
    def turn_right(self, velocity, angle):
	print("TURN")
	print("TURN VEL", velocity)	
	velocity = velocity / 2.0
	angle = -.5
	return velocity, angle

    def crawl(self, velocity, angle):
	print("WARNING: CRAWL")	
	velocity = velocity / 10.0
	angle = .5
	return velocity, angle



if __name__=='__main__':
    rospy.init_node("Explorer_node", anonymous=False)
    Explorer = Explore()
    rospy.sleep(0.1)
    rospy.spin()






