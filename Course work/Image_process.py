#!/usr/bin/env python
#
# This node synchonizes Image and CameraInfo messages for each RGB camera from the rosbag to the image processing node.  
#   Image processing node takes the image and a camera calibration message instead of setting camera calibration once at 
#   the start.  Publishing the camera calibration paramters led to a mismatch in time-stamps between the image and
#   calibration data, leading the image processing node to ignore both messages.  This
#   node will replace the image processing node and rectify the image.  Since the camera calibration does not 
#   change, this node pairs a CameraInfo message with each rectified images and publishes the pair
#

import rospy
import time
import math
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

class Republisher:
  def __init__(self):
    # Node
    rospy.init_node('camera_data_pub')

    self.r_rate = rospy.Rate(5)

    # Subscribers
    self.cam0 = rospy.Subscriber("/camera_array/cam0/image_raw", Image, self.repub_left)
    self.cam1 = rospy.Subscriber("/camera_array/cam1/image_raw", Image, self.repub_right)
    self.tf_sub = rospy.Subscriber("/tf_static", TFMessage, self.tf_repub)

    # Publishers
    self.left_cal_pub = rospy.Publisher('stereo_camera/cam0/camera_info', CameraInfo, queue_size=30)
    self.left_img_pub = rospy.Publisher('/stereo_camera/cam0/image_rect', Image, queue_size=30)
    self.right_cal_pub = rospy.Publisher('stereo_camera/cam1/camera_info', CameraInfo, queue_size=30)
    self.right_img_pub = rospy.Publisher('/stereo_camera/cam1/image_rect', Image, queue_size=30)
    self.tf_static_pub = rospy.Publisher('/tf', TFMessage, queue_size=30)

    self.bridge = CvBridge()

  def tf_repub(self, m):
    while not rospy.is_shutdown():
      self.tf_static_pub.publish(m)
      self.r_rate.sleep()
      
  def repub_left(self, cam0):
    rospy.loginfo("Republisher: repub_left")
    # LEFT CAMERA PARAMETERS
    # Resolution
    height = 1024
    width = 1224
    
    # Distortion Coefficients
    D1L = -0.03116674317579859
    D2L = 0.5005703117394807
    D3L = -7.691057047799489
    D4L = 41.71286545440132
    
    # Intrinsic Params
    fxl = 1888.4451558202136
    fyl = 1888.4000949073984
    cxl = 613.1897651359767
    cyl = 482.1189409211585
    
    ros_time_l = rospy.get_rostime()
    
    # Publish LEFT
    cal_left = CameraInfo()
    cal_left.header.frame_id = 'cam_0_optical_frame'
    cal_left.header.stamp.secs = ros_time_l.secs
    cal_left.header.stamp.nsecs = ros_time_l.nsecs
    cal_left.height = height
    cal_left.width = width
    cal_left.distortion_model = 'rational_polynomial'
    cal_left.D = [D1L, D2L, D3L, D4L]
    cal_left.K = fxl,0,cxl,0,fyl,cyl,0,0,1
    cal_left.R = 0,0,0,0,0,0,0,0,0
    cal_left.P = fxl,0,cxl,0,0,fyl,cyl,0,0,0,1,0 # use original cam matrix for raw image, cam 0 is origin, tx ty = 0
    cal_left.binning_x = 1
    cal_left.binning_y = 1
    cal_left.roi.width = 0
    cal_left.roi.height = 0

    # Publish left cal
    self.left_cal_pub.publish(cal_left)
    
    # Rectify LEFT IMAGE
    cameraMatrix = np.array([[fxl,0,cxl],[0,fyl,cyl],[0,0,1]])
    left_raw = self.bridge.imgmsg_to_cv2(cam0, desired_encoding='passthrough')
    #left_raw = cv2.imread(cam0.data)
    #left_raw = np.array(cam0.data)
    #left_raw = cam1.data
    left_rect = cv2.undistort(left_raw, cameraMatrix, (D1L,D2L,D3L,D4L), None, cameraMatrix)


    # Publish LEFT IMAGE
    img_left = Image()
    img_left.header.frame_id = 'cam_0_optical_frame'
    img_left.header.stamp.secs = ros_time_l.secs
    img_left.header.stamp.nsecs = ros_time_l.nsecs
    img_left.height = cam0.height
    img_left.width = cam0.width
    img_left.encoding = cam0.encoding
    img_left.step = cam0.step
    img_left.is_bigendian = cam0.is_bigendian
    #img_left.data = left_rect
    img_left.data = cam0.data

    # Publish left img
    # self.left_img_pub.publish(img_left)
    self.left_img_pub.publish(self.bridge.cv2_to_imgmsg(left_rect, encoding='passthrough'))
    
  def repub_right(self, cam1):
    rospy.loginfo("Republisher: repub_right")
    # RIGHT CAMERA PARAMETERS
    # Resolution
    height = 1024
    width = 1224
    
    # Distortion Coefficients
    D1R = -0.10081622359739374
    D2R = 2.439006534341921
    D3R = -26.79128779289829
    D4R = 101.51121325978683
    
    # Intrinsic Params
    fxr = 1868.5741276186334
    fyr = 1869.70165954517
    cxr = 573.2247250514644
    cyr = 460.0106709634189
    txl = -0.3288878477162784
    tyl = -0.0013499051732346753
    
    ros_time_r = rospy.get_rostime()
    
    # Publish RIGHT
    cal_right = CameraInfo()
    cal_right.header.frame_id = 'cam_1_optical_frame'
    cal_right.header.stamp.secs = ros_time_r.secs
    cal_right.header.stamp.nsecs = ros_time_r.nsecs  
    cal_right.height = height
    cal_right.width = width
    cal_right.distortion_model = 'rational_polynomial'
    cal_right.D = [D1R, D2R, D3R, D4R]
    cal_right.K = fxr,0,cxr,0,fyr,cyr,0,0,1
    cal_right.R = 0,0,0,0,0,0,0,0,0
    cal_right.P = fxr,0,cxr,txl,0,fyr,cyr,tyl,0,0,1,0 # use original cam matrix for raw image
    cal_right.binning_x = 1
    cal_right.binning_y = 1
    cal_right.roi.width = 0
    cal_right.roi.height = 0
    
    # Publish right cal
    self.right_cal_pub.publish(cal_right)
    
    # Rectify RIGHT IMAGE
    cameraMatrix = np.array([[fxr,0,cxr],[0,fyr,cyr],[0,0,1]])
    right_raw = self.bridge.imgmsg_to_cv2(cam1, desired_encoding='passthrough')
    #right_raw = cv2.imread(cam1.data)
    #right_raw = np.array(cam1.data)
    #right_raw = cam1.data
    right_rect = cv2.undistort(right_raw, cameraMatrix, (D1R,D2R,D3R,D4R), None, cameraMatrix)

    # Publish RIGHT IMAGE
    img_right = Image()
    img_right.header.frame_id = 'cam_1_optical_frame'
    img_right.header.stamp.secs = ros_time_r.secs
    img_right.header.stamp.nsecs = ros_time_r.nsecs
    img_right.height = cam1.height
    img_right.width = cam1.width
    img_right.encoding = cam1.encoding
    img_right.step = cam1.step
    img_right.is_bigendian = cam1.is_bigendian
    #img_right.data = right_rect
    img_right.data = cam1.data
    
    # Publish right img
    #self.right_img_pub.publish(img_right)
    self.right_img_pub.publish(self.bridge.cv2_to_imgmsg(right_rect, encoding='passthrough'))

if __name__ == '__main__':
  try:
    node = Republisher()
    rospy.loginfo("Initializing...")
    rospy.spin()
  except rospy.ROSException as e:
    rospy.logerr(e)
