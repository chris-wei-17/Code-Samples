#!/usr/bin/env python3
import pyzed.sl as sl
import cv2
import numpy as np
from PIL import Image
from matplotlib import pyplot as plt


class Tracker():

	def __init__(self):
	
		# Create a Camera object
		self.zed = sl.Camera()

	    # Create a InitParameters object and set configuration parameters
		init_params = sl.InitParameters()
		init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
		init_params.camera_fps = 30  # Set fps at 30
	
		# Open the camera
		err = self.zed.open(init_params)
		if err != sl.ERROR_CODE.SUCCESS:
			exit(1)

		# Initialize Variables
		self.rgb_r = None
		self.rgb_l = None
    
		print("init complete")    
    
	def read(self):

		zed_R = sl.Mat(self.zed.get_camera_information().camera_resolution.width, self.zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
		zed_L = sl.Mat(self.zed.get_camera_information().camera_resolution.width, self.zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
    
		runtime_parameters = sl.RuntimeParameters()
    
	    # Grab an image, a RuntimeParameters object must be given to grab()
		if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
	        # A new image is available if grab() returns SUCCESS
			self.zed.retrieve_image(zed_R, sl.VIEW.RIGHT)
			self.zed.retrieve_image(zed_L, sl.VIEW.LEFT)
	    
		image_R = zed_R.get_data()
		image_L = zed_L.get_data()
	    
		self.rgb_r = cv2.cvtColor(image_R, cv2.COLOR_RGBA2RGB)
		self.rgb_l = cv2.cvtColor(image_L, cv2.COLOR_RGBA2RGB)   
		
		print("read complete")
	
	#def define_roi():
	
	def orb_detect(self):
	
	    orb = cv2.ORB_create()
	    kp_r= orb.detect(self.rgb_r,None)
	    kp_r, des_r = orb.compute(rgb_r, kp_r)
	    kp_l= orb.detect(self.rgb_l,None)
	    kp_l, des_l = orb.compute(rgb_l, kp_l)    
	
	    img_r = cv2.drawKeypoints(rgb_r, kp_r, None, color=(0,255,0), flags=0)
	    img_l = cv2.drawKeypoints(rgb_l, kp_l, None, color=(0,255,0), flags=0)
	
	    #plot = plt.figure()
	    #plot.add_subplot(1,2,1)    
	    #plt.imshow(img_r)
	    #plot.add_subplot(1,2,2)
	    #plt.imshow(img_l)
	    #plt.show(block=True)
	    
	    bf = cv2.BFMatcher()
	    matches = bf.knnMatch(des_r,des_l,k=2)
	    
	    good = []
	    for m,n in matches:
	        if m.distance < 0.75*n.distance:
	            good.append([m])
	
	    # cv2.drawMatchesKnn expects list of lists as matches.
	    img3 = cv2.drawMatchesKnn(img_r,kp_r,img_l,kp_l,good,None,flags=2)
	
	    plt.imshow(img3)
	    plt.show()
	    cv2.imshow("match", img3)
	    cv2.waitKey(0)
	    
	    print("detect complete")
	    
	#def track():
	    
	    
	#def odometry():
	
	
if __name__ == "__main__":
	T = Tracker()
