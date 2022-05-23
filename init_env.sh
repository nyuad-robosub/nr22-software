
echo "Building ROS nodes"
# sudo apt update
# sudo apt install python3-opencv

# sudo apt-get install build-essential

# sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
# #cd ./catkin_ws
# p=$(pwd)
# mkdir requiredlibs
# cd requiredlibs

# git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
# git clone https://gitlab.com/libeigen/eigen.git

# # Get Pangolin
# cd Pangolin 
# mkdir build
# cmake .. 
# make

# #cd $p
# #cd ./requiredlibs/eigen

# #mkdir build
# #cd build
# #cmake ..
# #sudo make install 
# #sudo apt install libeigen3-dev

# # Install dependencies (as described above, or your preferred method)
# ./scripts/install_prerequisites.sh recommended

# # Configure and build
# #mkdir build && cd build
# #cmake ..
# #cmake --build .

# # GIVEME THE PYTHON STUFF!!!! (Check the output to verify selected python version)
# #cmake --build . -t pypangolin_pip_install

# # Run me some tests! (Requires Catch2 which must be manually installed on Ubuntu.)
# #ctest


# #FIX ORBSLAM ADD 
# #include <unistd.h>
# #include <stdio.h>
# #include <stdlib.h>
# cd $p
# cd ./src
# git clone https://github.com/nyuad-robosub/realsense-ros
# git clone https://github.com/nyuad-robosub/ORB_SLAM2

# sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
# sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
# sudo apt-get update
# sudo apt-get install librealsense2-dkms --allow-unauthenticated -y
# sudo apt-get install librealsense2-dev --allow-unauthenticated -y
# catkin config --extend /opt/ros/melodic
# cd $p
# catkin build || true
# cd $p
# source $p/devel/setup.bash

#to run realsense roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
#rosrun ORB_SLAM2 RGBD ~catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.txt ~catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Asus.yaml

git submodule update --init --recursive

#install orbslam3 dependencies
sudo apt install libgl1-mesa-dev libglew-dev cmake python3.6 libpython3.6-dev pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev libjpeg-dev libtiff5-dev libopenexr-dev python3-pip g++ git gcc
sudo apt install libssl-dev libopencv-dev libboost-filesystem-dev libboost-serialization-dev
sudo apt install python3-opencv

# Install realsense2 packages
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt-get update
sudo apt-get install librealsense2-dkms --allow-unauthenticated -y
sudo apt-get install librealsense2-dev --allow-unauthenticated -y

#install eigen IS THIS NEEDED?
#sudo apt install libeigen3-dev


cd ./src
p=$(pwd)


# Get Pangolin
cd $p/deps/Pangolin 
mkdir build
cd build
cmake .. 
sudo make install

#FOR ORBSLAM WE NEED SOPHUS #include <sophus/se3.hpp> DOESNT WORK
cd $p/deps/Sophus
mkdir build
cd build
cmake ..
sudo make install

#GET EIGEN 3.2.10 VERSION FOR NO CONFLICTS
# cd $p/deps
# wget "https://gitlab.com/libeigen/eigen/-/archive/3.2.10/eigen-3.2.10.tar.bz2"
# tar -xf eigen-3.2.10.tar.bz2
# cd $p/deps/eigen-3.2.10
# mkdir build
# cd build
# cmake ..
# make
# sudo make install
cd $p/deps
wget "https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.bz2"
tar -xf eigen-3.3.9.tar.bz2
cd $p/deps/eigen-3.2.10
mkdir build
cd build
cmake ..
make
sudo make install


#install opencv 4.4 
cd $p/deps
# Download and unpack sources
# Create build directory
mkdir -p build && cd build
# Configure
cmake  ..#/opencv-4.x
# Build
cmake --build .

# wget "http://bitbucket.org/eigen/eigen/get/3.2.8.tar.gz"

# tar zxvf 3.2.8.tar.gz

# cd eigen*

# /eigen-eigen-07105f7124f9$ mkdir build

# /eigen-eigen-07105f7124f9$ cd build

# /eigen-eigen-07105f7124f9/build$ cmake .. 

# /eigen-eigen-07105f7124f9/build$ make

# /eigen-eigen-07105f7124f9/build$ sudo make install


cd $p
catkin config --extend /opt/ros/melodic
catkin build || true #will not build realsense2_camera 

cd $p 
cd ..
p=${pwd}
source $p/devel/setup.sh

#create orbslam3 ros package separately

#cd $p/include/ORB_SLAM3/
bash $p/include/ORB_SLAM3/build_ros.sh
#source /opt/ros/melodic/setup.bash
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$WORKDIR/ORB_SLAM3/Examples/ROS/ PUT THIS IN BASH RC OR SOURCE DEVEL?


