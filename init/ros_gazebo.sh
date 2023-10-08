#!/bin/bash
# Script description
# | This script install ros-melodic-desktop-full as well as some other 
# | ROS packages needed for nr22-software. It also prompts you to add 
# | /opt/ros/melodic as default workspace. Should not be run within a
# | virtual environment, as some ROS packages use system Python to
# | install.

NR22_DIRECTORY=$PWD
# Add ROS to package list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# If curl is not installed yet
sudo apt-get install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install ROS + Gazebo
sudo apt-get update # && sudo apt-get upgrade
sudo apt-get install -y ros-melodic-desktop-full # ros-melodic-ros-gz # -full
sudo apt-get install -y ros-melodic-uuv-simulator ros-melodic-robot-localization ros-melodic-ddynamic-reconfigure ros-melodic-vision-msgs ros-melodic-tf2-sensor-msgs ros-melodic-rospy-message-converter ros-melodic-rgbd-launch ros-melodic-camera-info-manager ros-melodic-pcl-ros ros-melodic-image-transport ros-melodic-smach ros-melodic-rosbuild ros-melodic-mavros nlohmann-json-dev

# Install system python packages
sudo apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool python-pip python3-pip
sudo rosdep init && rosdep update 

# Verify Gazebo installation
gazebo --version
which gzserver
which gzclient
# Fixing Gazebo libignition
# https://answers.gazebosim.org/question/23475/ssl-no-alternative-certificate-subject-name-matches-target-host-name-apiignitionfuelorg/?answer=25944#post-id-25944
# sudo apt-get install -y --only-upgrade libignition-math2
sed -i 's/api.ignitionfuel.org/fuel.ignitionrobotics.org/' $HOME/.ignition/fuel/config.yaml

# Check if user wants to add ROS setup.bash into .bashrc (if doesn't exist yet)
SHELL_INIT=".bashrc"
exportline="source /opt/ros/melodic/setup.bash";
eval $exportline
grep -Fxq "$exportline" ~/$SHELL_INIT 2>/dev/null || {
    if utils_prompt_user "Add \"source /opt/ros/melodic/setup.bash\" to ~/.bashrc? (not neccessary if using catkin workspaces later)" n; then
        echo $exportline >> ~/$SHELL_INIT
        echo "Done!"
    fi
}

cd $NR22_DIRECTORY
