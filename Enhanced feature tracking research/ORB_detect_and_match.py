#!/usr/bin/env python3
import pyzed.sl as sl
import cv2
import numpy as np
from PIL import Image
from matplotlib import pyplot as plt


    
def main():
    # Create a Camera object
	zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
	init_params = sl.InitParameters()
	init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
	init_params.camera_fps = 30  # Set fps at 30

    # Open the camera
	err = zed.open(init_params)
	if err != sl.ERROR_CODE.SUCCESS:
		exit(1)

	zed_R = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
	zed_L = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
    
	runtime_parameters = sl.RuntimeParameters()
    
    # Grab an image, a RuntimeParameters object must be given to grab()
	if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # A new image is available if grab() returns SUCCESS
		zed.retrieve_image(zed_R, sl.VIEW.RIGHT)
		zed.retrieve_image(zed_L, sl.VIEW.LEFT)
    
	image_R = zed_R.get_data()
	image_L = zed_L.get_data()
    
    # Convert image format to RGB
	rgb_r = cv2.cvtColor(image_R, cv2.COLOR_RGBA2RGB)
	rgb_l = cv2.cvtColor(image_L, cv2.COLOR_RGBA2RGB)    

	# Define ROI boundaries. X,Y will eventually be defined by function driven by vehicle control
	X1 = 480
	X2 = 1440
	Y1 = 270
	Y2 = 810
	top_left = (X1,Y1)
	bottom_right = (X2,Y2)

	# Draw ROI for feature detection
	region_r = cv2.rectangle(rgb_r, top_left, bottom_right, (0,255,0), 3)
	region_l = cv2.rectangle(rgb_l, top_left, bottom_right, (0,255,0), 3)
	
	#plt.figure(1)
	#plt.imshow(region_r)
	#plt.figure(2)
	#plt.imshow(region_l)
	
	orb = cv2.ORB_create(nlevels=4, nfeatures=500, scoreType=cv2.ORB_HARRIS_SCORE)
	kp_r= orb.detect(rgb_r,None)
	kp_r, des_r = orb.compute(rgb_r, kp_r)
	kp_l= orb.detect(rgb_l,None)
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
	
	plt.figure(3)
	plt.imshow(img3)
	
	plt.show()
    
	zed.close()
 

if __name__ == "__main__":
	main()
