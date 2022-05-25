
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

# Install realsense2 packages
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt-get update
sudo apt-get install librealsense2-dkms --allow-unauthenticated -y
sudo apt-get install librealsense2-dev --allow-unauthenticated -y
sudo apt install ros-melodic-rgbd-launch

cd ./include
p=$(pwd)

# # Get Pangolin
# wget "https://codeload.github.com/stevenlovegrove/Pangolin/zip/refs/tags/v0.5"
# unzip v0.5
# rm v0.5
cd $p/Pangolin*
mkdir build
cd build
cmake .. 
sudo make install

#FOR ORBSLAM WE NEED SOPHUS #include <sophus/se3.hpp>
cd $p/Sophus
mkdir build
cd build
cmake ..
sudo make install

#FOR ORBSLAM WE NEED FMT #include <sophus/se3.hpp>
cd $p/fmt
mkdir build
cd build
cmake ..
sudo make install

#GET EIGEN 3.2.10 VERSION FOR NO CONFLICTS
# wget "https://gitlab.com/libeigen/eigen/-/archive/3.2.10/eigen-3.2.10.tar.bz2"
cd $p
#wget "https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.bz2"
#tar -xf eigen-3.3.9.tar.bz2
#rm eigen-3.3.9.tar.bz2

#wget "https://gitlab.com/libeigen/eigen/-/archive/3.3.0/eigen-3.3.0.tar.bz2"
#tar -xf eigen-3.3.0.tar.bz2
wget "https://gitlab.com/libeigen/eigen/-/archive/3.2.10/eigen-3.2.10.tar.bz2"
tar -xf eigen-3.2.10.tar.bz2
rm eigen-3.2.10.tar.bz2
cd $p/eigen*
mkdir build
cd build
cmake ..
#make
sudo make install


# #install opencv 4.4 DOWNGRADE
# cd $p/opencv
# # Download and unpack sources
# # Create build directory
# mkdir -p build && cd build
# cmake ..
# make
# sudo make install

cd $p
cd ..
p=$(pwd)
catkin config --extend /opt/ros/melodic
catkin build || true 

source $p/devel/setup.sh

#create orbslam3 ros package separately
#bash $p/include/ORB_SLAM3/build_ros.sh
bahs $p/include/ORB_SLAM2/build_ros.sh

#source /opt/ros/melodic/setup.bash
#export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/nr22-software/include/ORB_SLAM2/Examples/ROS/ORB_SLAM2
#roslaunch realsense2_camera rs_rgbd.launch initial_reset:=true
#rosrun ORB_SLAM2 RGBD /home/rami/nr22-software/include/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/rami/nr22-software/include/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Asus.yaml

