# Trash Bot Base on Turtlebot3 with open manipulator (Not perfected yet)
## Use Yolov8 to detect trash(bottle). And Apply PID control to adjust the robot position(make the trash box center equal with the image center). 
## Operate the Actual Robot
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

## Operate the Virtual Robot
``` bash
$ roslaunch trash_bot gazebo.launch
```
``` bash
$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```
``` bash
$ rosrun trash_bot arm_controller.py
```
