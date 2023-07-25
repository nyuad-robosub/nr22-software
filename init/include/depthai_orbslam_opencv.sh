#!/bin/bash
# Script description
# | This script installs depthai and ORBSLAM3 packages and their 
# | dependencies. The depthai package is needed for depthai_bridge
# | and depthai_examples.

# INSTALLS REQUIRED DEPENDENCIES AND BUILD CATKIN WORKSPACE

nr_directory=$(pwd)
include_directory="${nr_directory}/include"

# COLORS AND LOGGING FUNCTIONS
MAGENTA="\x1b[35m"
NO_COLOR="\x1b[0m"
log () {
    echo -e $MAGENTA "NR LOG: " $1 $NO_COLOR 
}

log "Building ROS nodes"
# git submodule update --init --recursive

# log "Installing ROS"
# sudo apt-get install -y ros-melodic-desktop-full
# echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
# sudo apt-get install -y ros-melodic-ddynamic-reconfigure ros-melodic-rgbd-launch ros-melodic-vision-msgs ros-melodic-camera-info-manager ros-melodic-pcl-ros ros-melodic-image-transport ros-melodic-tf2-sensor-msgs ros-melodic-smach ros-melodic-rosbuild ros-melodic-mavros nlohmann-json-dev

# Install ORB-SLAM3 dependencies
log "Installing ORB-SLAM3 dependencies"
sudo apt install -y libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt install -y libgl1-mesa-dev libglew-dev cmake python3.6 libpython3.6-dev pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev libjpeg-dev libtiff5-dev libopenexr-dev python3-pip g++ git gcc ros-melodic-tf2-geometry-msgs
sudo apt install -y libusb-1.0-0-dev

# sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# sudo apt-get install -y curl # if you haven't already installed curl
# curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

cd ./include
p=$(pwd)

pip2 install opencv-contrib-python

# SVO dep installation
sudo apt install -y libgoogle-glog-dev

# Get Pangolin
sudo apt-get install -y libglew-dev libboost-dev libboost-thread-dev libboost-filesystem-dev ffmpeg libavutil-dev libpng-dev
cd $include_directory/Pangolin*
mkdir build
cd build
cmake .. 
sudo make install

# #FOR ORBSLAM2 WE NEED SOPHUS #include <sophus/se3.hpp>
# cd $include_directory/Sophus
# mkdir build
# cd build
# cmake ..
# sudo make install

#FOR ORBSLAM3 WE NEED FMT #include <sophus/se3.hpp>
cd $include_directory/fmt
mkdir build
cd build
cmake ..
sudo make install

#JSON for depthai-ros
cd $include_directory/json
mkdir build
cd build
cmake ..
sudo make install

# EIGEN Version 3.2.10 FOR NO CONFLICTS
cd $include_directory
wget "https://gitlab.com/libeigen/eigen/-/archive/3.2.10/eigen-3.2.10.tar.bz2"
tar -xf eigen-3.2.10.tar.bz2
rm eigen-3.2.10.tar.bz2
cd $include_directory/eigen*
mkdir build
cd build
cmake ..
sudo make install

# Install OpenCV 4.4(If you have dependency issues)
# log "Configuring and building openCV for ORBSLAM3 and depthai-ros"
# cd $include_directory
# wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.4.0.zip
# unzip opencv
# rm opencv.zip
# cd $include_directory/opencv-4.4.0
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make
# Alternatively....
sudo apt install -y libopencv-dev python3-opencv

# Install depthai-core (if this has any issues try clearing the .Hunter folder in home directory)
log "Configuring/building depthai-core"
cd $include_directory/depthai-core
cmake -H. -Bbuild -D'BUILD_SHARED_LIBS=ON'
cmake --build build

# cd $nr_directory
# catkin config --extend /opt/ros/melodic
# catkin_make -j2
# catkin build

# source $nr_directory/devel/setup.sh
# echo $nr_directory'/devel/setup.sh' >> ~/.bashrc

#create orbslam3 ros package separately
cd $include_directory/ORB_SLAM3_OPENCV4
bash ./build.sh

# This is to add package path to bashrc for ROS support
# Fix the path
echo 'export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:'$include_directory'/ORB_SLAM3_OPENCV4/Examples/ROS/ORB_SLAM3' >> ~/.bashrc 
# source $nr_directory/devel/setup.bash
# echo 'source '$nr_directory'/devel/setup.bash' >> ~/.bashrc
bash ./build_ros.sh

cd $nr_directory

# CXX version fix 
# sed -i 's/++11/++14/g' CMakeLists.txt
