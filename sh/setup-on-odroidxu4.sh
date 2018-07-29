#!/bin/sh
cd ~
echo "export LC_ALL=C" >> ~/.bashrc
source ~/.bashrc

sudo apt-get update
sudo apt-get install python-pip -y
sudo pip install pymap3d==1.6.3
sudo pip install lap

# The following is for pymavlink
sudo apt-get install libxml2-dev libxslt-dev python-dev -y
sudo apt-get install python-lxml -y
sudo pip install future
sudo pip install pymavlink

sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools -y

mkdir ~/scripts
cd ~/catkin_ws/src/formation/scripts
cp startup_launch.sh ~/scripts/
cd ~/scripts
chmod +x startup_launch.sh

cd ~/catkin_ws
catkin build

# copy and enable the service file
cd /lib/systemd/system
sudo cp ~/catkin_ws/src/formation/sh/formation.service .
sudo systemctl daemon-reload
sudo systemctl enable formation.service

cd ~
