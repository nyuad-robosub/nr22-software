<p align="center">
  <a href="https://robosub.nyuad.io/">
    <img
      alt="NYUAD Robosub Logo"
      src="https://robosub.nyuad.io/wp-content/uploads/2022/06/Dugong-lines_v12-1-2048x1307.png"
      width="300"
    />
  </a>
</p>

<div align="center">
  <h1><a href="https://robosub.nyuad.io/"> NYUAD Robosub</a> | nr22-software</h1>
  <p>
    Repository used to operate the Dugong AUV in simulation and on-site operations for the NYUAD Robosub 2022 software team.
  </p>
</div>

# Table of contents
* [Repo structure](#repository-structure)
* [Installation](#installation)
  * [Script installation](#script-installation)
  * [Manual installation](#manual-installation)
* [Usage](#usage)
  * [Launching simulation](#running-simulation-scripts)
  * [Launching ORB_SLAM3 with OAK-D](#launching-orb_slam3-with-the-oak-d)
  * [Installing ORB_SLAM2](#installing-orb_slam2-and-oakd-ros-launch-files)
  * [Launching ORB_SLAM2](#running-orb-slam2-and-oak-d-pipeline-scripts)
* [Development](#development)
  * [Creating new packages](#creating-new-packages)

# Repository structure
The repository is structured as a ROS catkin workspace with the following directories:

    .
    ├── include                 # Submodules and libraries that are not ROS packages needed to test and run the software
    ├── src                     # Files containing all simulation and on-site scripts
    │   ├── avoid_obstacles     # Catkin package: Path planning scripts based off pointclouds
    │   ├── camera_controller   # Catkin package: OAK-D camera controller used to publish stereo camera data/swap calibration files
    │   ├── color_detection     # Scripts used to detect colors underwater
    │   ├── controls            # Catkin package: Sends telemetry data and TF2 frames of robot in space for navigation
    │   ├── deps                # Repository submodules containing all external ROS package dependencies
    │   ├── edge-gate-detection # Competition gate detection scripts
    │   ├── faux_detection      # Catkin package: Sends machine learning object detections for simulation purposes.
    │   ├── handle_detection    # Bin task handle detection scripts
    │   ├── mission_planning    # Catkin package: Main mission planning scripts used for both simulation/on-site runs.
    │   ├── pools               # Catkin package: Contains Gazebo world files for pool simulation
    │   ├── sonar_ping          # Catkin package: Sends sonar pinger data for simulation
    │   ├── torpedo-hole        # Torpedo task detection scripts
    └── README.md

# Installation
The repository uses ROS Melodic due to legacy issues, and as of now can only be installed on Ubuntu 18.04 or similar systems.

## Script installation
The initialization scripts are located in the `init` folder and is structured similarly to the repository modules. If the default flag `-d` is not set, the main script `init_env.sh` will describe each initialization script and prompt for installation.

- Clone nr22-software and all submodules (clone outside of pre-existing ROS workspaces): 

`git clone https://github.com/nyuad-robosub/nr22-software --recursive`

- **`cd`** into it. This is very important as the initialization scripts will rely on `$PWD` to understand directories.

`cd nr22-software`

- For a fresh Ubuntu 18.04 or similar system,  run the bash script with **`source`** and the default tag `-d`:

`source init_env.sh -d`

The installation may fail at some initialization scripts due to build errors (out of memory, permission issues, etc.) If so, use the following step to rerun the failed script/run the next scripts.

- For more control of the installation, run the script with `source` and no `-d`:

`source init_env.sh`

The installation will prompt you for each initialization script, as well as some important decisions within each script. All decisions with a default option can be answered with Enter instead of `y`/`n`.

## Manual installation

- Clone the nr22-software and cd into it: (clone it outside of pre-existing workspaces, if any)

`git clone https://github.com/nyuad-robosub/nr22-software` (add `-b <branch>` if cloning a specific branch, and add `--single-branch` if only want to fetch single branch)

- Change directory to the new repo

`cd nr22-software`

- For first time cloning, init all the submodules:

`git submodule update --init --recursive`

- For later, to update submodules to latest version:

`git submodule update --recursive --remote`

- If there are no pre-existing catkin workspaces (only ROS exists on the system): extend `nr22-software` to ROS Melodic install folder

  - `catkin config --init --extend /opt/ros/melodic`

- If there is already a workspace (such as `~/gz_ws`) the extend above needs to be extending to that instead (note the `/devel` at the end). This is to "chain" all the workspaces, so that the build system can read all packages in all workspaces:

  - `catkin config --init --extend <path-to-last-workspace/devel>`
  - Example: `catkin config --init --extend ~/gz_ws/devel`

- Build the packages inside (this command can be run anywhere inside the `nr22-software` folder):

`catkin build` (to build specific packages use `catkin build <package-name>`)

- Source the new workspace after building (this needs to be done after a new package is built, everytime a terminal is opened, for ROS to recognize it):

`source devel/setup.bash`

- If you don't want to do that everytime, do this:
  - Go to `/home/<user>/`, Ctrl-H to see hidden files and open .bashrc (all opened terminals run this file initially)
  - Append `source <path-to-nr22-software>/nr22-software/devel/setup.bash` at the end of the file
  - This will allow each new terminal opened to recognize the new workspace and use the packages

# Usage
## Running simulation scripts

> Prior to running these scripts you should have run the init_env.sh file and [setup object detection here](src/faux_detection/README.md) (this is done if you did the [Script installation](#script-installation) route.)
- To run simulation, type `roslaunch mission_planning start_run_sim.launch`

## Launching ORB_SLAM3 with the OAK-D

- Simply start the launch file

`roslaunch ORB_SLAM3 oakd_node.launch`

## Installing ORB_SLAM2 and OAKD ros launch files


- To setup ORB_SLAM2:
- Go to `.bashrc`
- Append `export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/nr22-software/include/ORB_SLAM2/Examples/ROS/ORB_SLAM2`

## Running ORB-SLAM2 and OAK-D pipeline scripts:

- Type `rosrun ORB_SLAM2 STEREO_OAKD {PATH TO VOCABULARY FILE} {PATH TO CAMERA YAML CONFIGURAITON FILE}`
  - For example: `rosrun ORB_SLAM2 STEREO_OAKD /home/rami/nr22-software/include/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/rami/nr22-software/include/ORB_SLAM2/Examples/ROS/ORB_SLAM2/OAKD.yaml`

# Development
## Creating new packages

- References: https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html

- Change directory to `/src` (this is where all the packages are, can also be put in subfolders):

`cd src`

- Use catkin-tools to create packages. This will create a new package folder in `/src` that has your necessary files.

`catkin create pkg <package-name>`

`cd <package-name>`

- Rule of thumb: add files to their respective folders, such as `/scripts` for C++ and Python node files, `/launch` for launch files, etc.

- Refer to [this (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) and [this (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) to write your first nodes.

