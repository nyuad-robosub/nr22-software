#!/bin/bash
NR22_DIRECTORY=$PWD
INIT_DIRECTORY="$NR22_DIRECTORY/init"

# Default argument (-d) is passed directly to utils.sh because of `source`
# DEF_ARG=$1
source $INIT_DIRECTORY/utils.sh

# Array of all the init scripts in execution order
declare -a init_scripts=(
    "ros_gazebo.sh"
    "miniconda.sh"
    "include/depthai_orbslam_opencv.sh"
    "src/deps/realsense.sh"
    "src/deps/svo.sh"
    "catkin.sh"
    "include/ardupilot.sh"
    "src/faux_detection.sh"
)

# Loop through array to run scripts
for i in "${init_scripts[@]}"
do
    echo # New line for new file
    # Formatting courtesy of: https://stackoverflow.com/a/2924755
    echo "Description of $(tput bold)$i$(tput sgr0):"
    echo "============================================================"
    # Regex courtesy of: https://stackoverflow.com/a/1247828
    sed -n 's/^#\s*|\s*//p' $INIT_DIRECTORY/$i
    echo "============================================================"
    if utils_prompt_user "Run $i?" y; then
        source $INIT_DIRECTORY/$i
    fi
done
