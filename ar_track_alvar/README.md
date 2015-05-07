# Overview
This package originally comes from [ar_track_alvar](https://github.com/sniekum/ar_track_alvar/tree/hydro-devel). It can detect the pose of either a single ar tag or a bundle of ar tags using Kinect sensor. 

# Requirements
 - Kinect : If you choose to use the Kinect connected to the turtlebot, you need to *roslaunch turtlebot_bringup 3dsensor.launch depth_registration:=false* to get sensor topic data. 

# Lessons Learned
There are several things you should pay attention to when using this package. 
   1. Due to the limited processing power of the netbook, the frequency of detecing the ar tag is very low. This is further worsen by the topic (/camera/depth_registered/points) that is needed to recover the pose of ar tag.
   2. This package is more suitable for get the pose information in 3D world instead of 2D. If you want to query the transformation information in 2D fast enough, please try *cmvision* (maybe *cmvision_3d* works too).
