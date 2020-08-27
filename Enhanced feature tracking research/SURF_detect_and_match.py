#!/usr/bin/env python3
import pyzed.sl as sl
import cv2
import numpy

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

    image_R = sl.Mat()
    image_L = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    
    # Grab an image, a RuntimeParameters object must be given to grab()
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # A new image is available if grab() returns SUCCESS
    	zed.retrieve_image(image_R, sl.VIEW.RIGHT)
    	zed.retrieve_image(image_L, sl.VIEW.LEFT)
    	
    zed.close()

    surf = cv2.SURF(400)
    kp_r, des_r = surf.detectAndCompute(image_R,None)
    kp_l, des_l = surf.detectAndCompute(image_L,None)
    
    img = cv2.drawKeypoints(image_R,kp_r,None,(255,0,0),4)
    
    plt.imshow(img)
    plt.show()
    

if __name__ == "__main__":
    main()
