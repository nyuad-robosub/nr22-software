#!/bin/bash
NR22_DIRECTORY=$PWD

# Array of all the init scripts in execution order
INIT_DIRECTORY="$NR22_DIRECTORY/init"
declare -a init_scripts=(
    "ros_gazebo.sh"
    "miniconda.sh"
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
    echo # New line for new file
    read -p "Run $INIT_DIRECTORY/$i? [N/y]"
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        source $INIT_DIRECTORY/$i
    fi
done
