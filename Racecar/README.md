# Racecar
Northeastern University Masters program - autonomous vehicle project

This projects is an indpedent study being completed as a part of the Mechanical Engineering: Mechatronics program at Northeastern University


<img src="https://github.com/chris-wei-17/Code-Samples/blob/master/Racecar/Images/car.jpg" height="500" width="500">


The vehicle used in this project is based off of the MIT Racecar platform.  Hardware includes an Nvidia Jetson TX2, Stereolabs ZED camera, open source motor controller, and Traxxas RC car base.

ROS is used and additional software is written in python and c++.  The car is capable of using the odometry calculated by the ZED stereo camera as well as calculating odometry on-board using the stream of stereo images. The RTABMap library is used to enable 2D and 3D mapping.  Examples of map generation are shown in the videos below.

  
------2D grid map generation------

[![Watch the video](https://img.youtube.com/vi/uT_HKJX0hqE/0.jpg)](https://youtu.be/uT_HKJX0hqE)
 

------3D cloud map generation------

[![Watch the video](https://img.youtube.com/vi/kwg5O1kBDN8/0.jpg)](https://youtu.be/kwg5O1kBDN8)

  
Cloned repos/submodules were flattened to avoid cloning/commit issues that occured when testing the project on different computers.  Since this project is short-term, we are not worried about continuous improvement and maintaining updates with master repos.  Repo owner credit is maintained in respective readme files
