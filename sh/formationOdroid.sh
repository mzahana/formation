#!/bin/sh
cp ~/formation/sh/formationOdroid.sh ~
cd ~
chmod +x ./formationOdroid.sh
./formationOdroid.sh
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
mv ~/formation/* ./formation/
cd ..
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
