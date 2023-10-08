#!/bin/bash
# Script description
# | This script installs the RealSense packages for the realsense
# | ROS packages.

# Install realsense2 packages
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt-get update
sudo apt-get install -y librealsense2-dkms librealsense2-dev librealsense2-utils
