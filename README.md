# Trash Bot Base on Turtlebot3 with open manipulator
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
