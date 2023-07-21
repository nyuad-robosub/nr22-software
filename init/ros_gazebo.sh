#!/bin/bash
# Add ROS to package list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# If curl is not installed yet
sudo apt-get install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install ROS + Gazebo
sudo apt-get update && sudo apt-get upgrade
sudo apt-get install -y ros-melodic-desktop-full # ros-melodic-ros-gz # -full
sudo apt-get install -y ros-melodic-uuv-simulator ros-melodic-robot-localization ros-melodic-ddynamic-reconfigure ros-melodic-vision-msgs ros-melodic-tf2-sensor-msgs ros-melodic-rospy-message-converter

# Install system python packages
sudo apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool python-pip python3-pip
sudo rosdep init && rosdep update 

# Fixing Gazebo libignition
sudo apt upgrade libignition-math2
gazebo --version
which gzserver
which gzclient

# Check if user wants to add ROS setup.bash into .bashrc (if doesn't exist yet)
SHELL_INIT=".bashrc"
exportline="source /opt/ros/melodic/setup.bash";
eval $exportline
grep -Fxq "$exportline" ~/$SHELL_INIT 2>/dev/null || {
    read -p "Add \"source /opt/ros/melodic/setup.bash\" to ~/.bashrc? \
    (not needed if using catkin workspaces) [N/y]"
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo $exportline >> ~/$SHELL_INIT
        echo "Done!"
    fi
}
