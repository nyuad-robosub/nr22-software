#!/bin/bash
# Script description
# | This script installs catkin_tools and relevant packages into your
# | (preferably) Python2 environment. Can be used in both system and
# | virtual environments.

NR22_DIRECTORY=$PWD
# Update cmake version
pip install --upgrade pip
pip install --upgrade cmake

# Setup nr22-software as a catkin workspace
pip install catkin_tools rospkg defusedxml pymap3d==1.5.2 transforms3d empy psutil
catkin config --init --extend /opt/ros/melodic

# Run catkin_build few times to smooth out incorrect package orders
catkin build --mem-limit 50%
catkin build --mem-limit 70%
catkin build

# Check if user wants to add devel/setup.bash into .bashrc (if doesn't exist yet)
SHELL_INIT=".bashrc"
exportline="source $NR22_DIRECTORY/devel/setup.bash";
eval $exportline
grep -Fxq "$exportline" ~/$SHELL_INIT 2>/dev/null || {
    if utils_prompt_user "Set $NR22_DIRECTORY as default catkin workspace?" y; then
        echo $exportline >> ~/$SHELL_INIT
        echo "Done!"
    fi
}

cd $NR22_DIRECTORY
