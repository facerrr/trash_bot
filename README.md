# Trash Bot Base on Turtlebot3 with open manipulator
## Use Yolov8 to detect trash(bottle). And Apply PID control to adjust the robot position(make the trash box center equal with the image center). 
## Before start 1. Prepare everything related to turtlebot3 2. Build map for environment 3. Need to fix the position of the realsense camera and calculate the transformation matrix with the file common/transformation.py
## Operate the Actual Robot. Open a new terminal for each command
``` bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
``` bash
$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```
``` bash
$ rosrun trash_bot cleanning.py
```
``` bash
$ rosrun trash_bot RGBD_by_pipe.py
```



