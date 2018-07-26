#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/odroid/catkin_ws/devel/setup.bash
# Define ROS_MASTER_URI if you are using single ROS network
# If you are using telem radio to communicate with GCS, comment the following line
#export ROS_MASTER_URI=http://192.168.0.105:11311
#ip=`ifconfig wlan0 | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p'`
export ROS_HOSTNAME=$(hostname -I)
roslaunch formation start_robot_and_serial_nodes.launch
# Comment the above line, and uncomment the following if you are using serial telem radio
#roslaunch formation start_robot_and_serial_nodes
