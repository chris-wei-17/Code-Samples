---- Sample Descriptions ----

GPS_driver.py:  This is a ROS driver to enable the use of a USB GPS receiver. 

-----------------------------------------------------

IMU_driver.py: This is a ROS driver to enable the use of a USB IMU.  The driver provides both
	RPY and quaternion orientations

-----------------------------------------------------

Image_process.py: This is a synchronization and image processing ROS node.  Due to being move to remote classes, we were unable to complete the final project using the usual hardware.  Instead we were provided datasets to use, which created some minor problems.  In this instance,  camera images had to be re-synchronized with the calibration parameter messages before they could be processed and re-published.

