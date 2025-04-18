#! /bin/bash

sudo apt-get update
sudo apt-get -y install python-is-python3
sudo apt-get install -y build-essential vim emacs git tmux python3-dev curl python3-pip cmake libgif-dev openssh-server

# class code
mkdir src
cd src
git clone --recursive https://github.com/robotic-picker-sp22/fetch-picker.git

# ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full
sudo apt install -y python3-rosdep
cd ..
sudo rosdep init
rosdep update
sudo apt-get install -y ros-noetic-moveit-*
sudo apt-get install -y ros-noetic-fetch-*
sudo apt-get install -y ros-noetic-gazebo-ros-control
sudo apt-get install -y ros-noetic-grasping-msgs
sudo apt-get install -y python-wstool 
sudo apt-get install -y python3-catkin-tools 
sudo apt-get install -y python-rosinstall

# NVM
curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.8/install.sh | bash
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" # This loads nvm
nvm install node
npm install -g polymer-cli bower

# Caddy
curl https://getcaddy.com | bash -s personal http.cors

# build
source /opt/ros/noetic/setup.bash
catkin build

# setup bashrc
echo "\n#ros" >> ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
