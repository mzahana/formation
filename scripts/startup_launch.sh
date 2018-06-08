#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/odroid/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.0.105:11311
ip=`ifconfig wlan0 | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p'`
export ROS_HOSTNAME=$ip
roslaunch formation mavros_and_robot_nodes.launch
