#!/bin/bash
NR22_DIRECTORY=$PWD
cd $NR22_DIRECTORY/src/deps/svo
# System dependencies
sudo apt-get install -y libglew-dev libopencv-dev libyaml-cpp-dev libblas-dev liblapack-dev libsuitesparse-dev

# Install SVO
pip install vcstool==0.2
vcs-import < ./rpg_svo_pro_open/dependencies.yaml
touch minkindr/minkindr_python/CATKIN_IGNORE
# vocabulary for place recognition
cd rpg_svo_pro_open/svo_online_loopclosing/vocabularies && ./download_voc.sh
cd $NR22_DIRECTORY
