#!/bin/bash
# Install ROS packages
sudo apt-get install -y ros-melodic-mavros ros-melodic-geographic-msgs ros-melodic-geodesy
sudo bash /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh

# -- ArduPilot --
# Install ardupilot
NR22_DIRECTORY=$PWD
cd $NR22_DIRECTORY/include/ardupilot
git submodule update --init --recursive

# Install dependencies
# NOTE: This will install Python 2.7 packages into the current environment,
#   so changing Python environment could break operations like pymavlink,
#   sim_vehicle.py etc.
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Check if user wants to create automated bash file in ~/
cd $HOME
read -p "Create AS.sh in $HOME (~) for SITL simulations? [N/y]"
if [[ $REPLY =~ ^[Yy]$ ]]
then
    echo "#!/bin/bash
    source $HOME/.bashrc
    sim_vehicle.py -f gazebo-bluerov2 -v ArduSub -L Saadiyat --out=udp:0.0.0.0:14550 --out=udp:0.0.0.0:14551 --out=udp:0.0.0.0:14552 --out=udp:0.0.0.0:14553 --out=udp:0.0.0.0:14554 --out=udp:0.0.0.0:14555 --console
    " >> $HOME/AS.sh
    chmod u+x $HOME/AS.sh 
    echo "Done!"
fi

# -- ardupilot_gazebo --
# https://github.com/patrickelectric/ardupilot_gazebo/tree/add_link#usage-
cd $NR22_DIRECTORY/include/ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4 || true
sudo make install

# -- QGroundControl --
# Check if user wants to install QGroundControl
# https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html
cd $HOME
read -p "Download QGroundControl in $HOME (~)? [N/y]"
if [[ $REPLY =~ ^[Yy]$ ]]
then
    # sudo usermod -a -G dialout $USER # This is already done in install-prereqs-ubuntu.sh
    sudo apt-get remove modemmanager -y
    sudo apt-get install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
    sudo apt-get install libqt5gui5 -y
    sudo apt install libpulse-mainloop-glib0

    # If wget is not installed yet
    sudo apt-get install -y wget
    # wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
    wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.0.11/QGroundControl.AppImage
    chmod +x ./QGroundControl.AppImage
    echo "Done!"
fi

# Return to nr22-software
cd $NR22_DIRECTORY
