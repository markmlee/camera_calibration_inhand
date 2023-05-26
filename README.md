![Screenshot](handeye_calibration.png)


## Overview
Takes set of [RGB image, End-effector pose] from realsense D435 camera mounted on xArm Robot Arm, to solve the Ax = xB calibration problem using the ChAurco board where x = extrinsic calibration.
Manually move the robot through guide-mode or manual mode to put the robot in different positions, hit Enter to record, and repeat.

## Install
- python3 and compatiable opencv version to run ChAruco functions. Best found by running it and resolving the missing libaries.
```
$ pip install opencv-contrib-python

```
- VISP ros node package to solve Ax=Bx given pair of [cam,ee pose]. Refer to http://wiki.ros.org/visp_hand2eye_calibration
```
$ sudo-apt get install ros-$ROS_DISTRO-visp-hand2eye-calibration

```
- realsense SDK package. Refer to https://github.com/IntelRealSense/realsense-ros. If installed correctly, should be able to roslaunch realsense2 file
```
$ roslaunch realsense2_camera rs_aligned_depth.launch

```

confirm correct installation of opencv by importing
```
python
import cv2
```

This should not return error. If you encounter `AttributeError: partially initialized module 'cv2' has no attribute 'gapi_wip_gst_GStreamerPipeline' (most likely due to a circular import)`, this is because of version mistmatch. Downgrade to working version by
```
pip uninstall opencv-contrib-python
pip uninstall opencv
pip install opencv-contrib-python==4.6.0.66

```


## Set up
- Place a ChAruco board in FOV of xArm Robot arm with realsense. ChAruco board can be generated from https://calib.io/pages/camera-calibration-pattern-generator.
- xArm robot is turned on and the xArm_bringup ROS_pkg is up and running by calling below command. xArm joint /tf topics should be published and visible on RViz. 


-Roslaunch camera and update the camera K intrinsic matrix in the yaml file
```
roscd realsense2_camera/launch/
roslaunch rs_aligned_depth.launch 
rostopic echo /camera/color/camera_info (in a separate tab)
``` 

## Running
- Modify the yaml file to adjust the intrinsic camera calibration info. You can do this by rosecho calling the topic /camera/color/camera_info.
- Modify the ChAruco.py file init function to adjust the CharucoBoard_create() parameters with the ChAruco board dimensions you printed.  
- Make sure xArm is powered up and xArm ROS state publisher is running, as well as the realsense ROS node (verify by $rostopic list) 

-Bring up the SDK to manually move the xArm through GUI
 ```
cd /home/marklee/IAM/xArm
./UfactoryStudio-client-linux-1.0.1.AppImage

```

-Bring up the robot state publisher
```
cd /home/marklee/catkin_ws/src/xarm_ros/xarm6_moveit_config/launch
roslaunch realMove_exec_CMU.launch
```

-Run the manual script to collect EE and cam pose data
```
$ python main.py

```
- This code will put Franka into Guide Mode. Manually hand move the robot to 15 different positions, press enter in between each position to store data.
- Given the generated two .csv files, modify publisher.py file if needed to see the path to the two .csv files are correct and can be loaded.
- Run the VISP server that will print out camera extrinsic info upon a rosservice call to publish the data. The sequence is as follows in separate terminals:

```
$ rosrun visp_hand2eye_calibration visp_hand2eye_calibration_calibrator
$ python publisher.py
$ rosservice call compute_effector_camera

```
The calibrator node will subscribe to the published [cam,ee pose pairs], then upon a service call, print out the extrinsic calibration matrix.
