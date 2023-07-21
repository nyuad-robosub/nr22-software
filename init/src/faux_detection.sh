#!/bin/bash
# -- object_detection --
NR22_DIRECTORY=$PWD
cd $NR22_DIRECTORY/include/models/research
pip install tensorflow==1.15 Cython contextlib2 pillow lxml
python setup.py build

# Check if user wants to install with --user
read -p "Run setup.py install with --user? \
(--user can lead to issues in virtual environments but is needed if installing with system Python) [N/y]"
if [[ $REPLY =~ ^[Yy]$ ]]
then
    python setup.py install --user
else
    python setup.py install
fi

# Run protoc to generate files
sudo apt-get install -y protobuf-compiler
protoc object_detection/protos/*.proto --python_out=.

# Check if user wants to add $PWD into PYTHONPATH (if doesn't exist yet)
SHELL_LOGIN=".profile"
exportline="export PYTHONPATH=\$PYTHONPATH:$PWD:$PWD/slim";
grep -Fxq "$exportline" ~/$SHELL_LOGIN 2> /dev/null || {
    read -p "Add $PWD and $PWD/slim to PYTHONPATH? \
    (needed for object_detection to find proto files properly) [N/y]"
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo $exportline >> ~/$SHELL_LOGIN
        eval $exportline
        echo "Done!"
    fi
}

cd $NR22_DIRECTORY
