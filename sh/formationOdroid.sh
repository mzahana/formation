#!/bin/sh
cd ~
sudo rm /var/lib/dpkg/lock
sudo apt-get install vim -y
sudo apt-get install curl -y
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
sudo python get-pip.py
sudo pip install pymap3d
sudo pip install lap
mkdir -p ./catkin_ws/src
cd ./catkin_ws/src
catkin_init_workspace
wstool init
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
mv ~/formation ./formation
mkdir ./formation/src
cd ..
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://192.168.0.105:11311" >> ~/.bashrc
ip=`ifconfig wlan0 | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p'`
echo "export ROS_HOSTNAME=$ip" >> ~/.bashrc
