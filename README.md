ROS adapter for the EPFL Ranger robot
=====================================

This ROS package provides:

- a set of launch files suitable to drive around the robot  and to publish RGB-sensor data
  (assuming a primesense or asus xtion sensor)
- a node (`ros_sensors`) that expose ROS topics for the robot sensors
  (odometry as `odom`, battery voltage and level, scale sensor data, the TF transformation for `base_link`, etc.)


![ROS mapping with the Ranger](ranger_mapping.png "gmapping on the Ranger, viewed in RViz")

Usage
------------
To bringup ranger motor control and robot sensor data publishing:
```
roslaunch ranger_ros bringup.launch
```
To bringup 3D sensor on the robot with fake laser scan from depth:
```
roslaunch ranger_ros 3dsensor.launch
```

Robot can be moved around in the environment using ROS package `teleop_twist_keyboard` (firstly install it using `sudo apt-get install ros-"your ros distro"-teleop_twist_keyboard`. Corresponding node can be launched using:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

To bring Ranger URDF model with ROS RVIZ from `ranger_description` package:
```
roslaunch ranger_description display.launch
```
Dependencies
------------

- [`ranger_description`](https://github.com/severin-lemaignan/ranger_description)
- [`pyranger`](https://github.com/chili-epfl/pyranger)
- [`openni2_launch`](http://wiki.ros.org/openni2_launch)
- [`depthimage_to_laserscan`](http://wiki.ros.org/depthimage_to_laserscan)

+ ROS 2D SLAM and navigation tools (`gmapping`, `amcl`, `move_base`...)

Technical checks
----------------
- Launch file `bringup.launch` is configured to work with the `mobots24` local wifi network, where IP of the robot is static (10.0.0.190) and it is written inside launch file. If network is changed, do not forget to change IP address as well.
- Checking the odometry is ok: run `rqt_plot` + Ranger's `teleop`, plot `/cmd_vel/linear/x`, `/cmd_vel/angular/z`, `/odom/twist/linear/x` and `/odom/twist/linear/z` and check everyone match.
