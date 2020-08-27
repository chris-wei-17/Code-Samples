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
    
    
    #cv2.imshow("Right Frame", image_R)
    #plt.imshow(image_R)   
    #plt.imshow(image_L)
    #plt.show

    zed.close()


if __name__ == "__main__":
    main()


