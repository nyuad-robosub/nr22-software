#!/bin/bash
# Setup nr22-software as a catkin workspace
NR22_DIRECTORY=$PWD
pip install --upgrade pip
pip install catkin_tools pymap3d==1.5.2 transforms3d
catkin config --init --extend /opt/ros/melodic

# Run catkin_build 4 times to smooth out incorrect package orders
catkin build
catkin build
catkin build
catkin build

# Check if user wants to add devel/setup.bash into .bashrc (if doesn't exist yet)
SHELL_INIT=".bashrc"
exportline="source $NR22_DIRECTORY/devel/setup.bash";
eval $exportline
grep -Fxq "$exportline" ~/$SHELL_INIT 2>/dev/null || {
    read -p "Set $NR22_DIRECTORY as default catkin workspace? [N/y]"
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo $exportline >> ~/$SHELL_INIT
        echo "Done!"
    fi
}

cd $NR22_DIRECTORY
