#!/bin/bash
# Update all submodules
git submodule update --init --recursive
NR22_DIRECTORY=$PWD

# Array of all the init scripts in execution order
INIT_DIRECTORY="init"
declare -a init_scripts=(
    "miniconda.sh"
    "ros_gazebo.sh"
    "src/deps/realsense.sh"
    "src/deps/svo.sh"
    "catkin.sh"
    "include/orbslam_opencv.sh"
    "include/ardupilot.sh"
    "src/faux_detection.sh"
)

# Loop through array to run scripts
for i in "${init_scripts[@]}"
do
    read -p "Run $INIT_DIRECTORY/$i? [N/y]"
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        bash $INIT_DIRECTORY/$i
    fi
done
