# nr22-software
Repository for Robosub 2022 software team.

The repository is structured as a catkin workspace with only the `/src` directory (ignoring all of the catkin files and folders).

## Installing

- Clone the nr22-software and cd into it: (clone it outside of pre-existing workspaces, if any)

`git clone https://github.com/nyuad-robosub/nr22-software` (add `-b <branch>` if cloning a specific branch, and add `--single-branch` if only want to fetch single branch)

- Change directory to the new repoL

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

##Installing(Alternative)

-Change directory to the new repo:

`cd nr22-software`

-Then run the bash script:

`sudo bash ./init_env.sh`

##Installing ORB_SLAM2 and OAKD ros launch files

-To setup ORB_SLAM2:
 - Go to `.bashrc` 
 - Append `export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/nr22-software/include/ORB_SLAM2/Examples/ROS/ORB_SLAM2`

## Creating new packages

- References: https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html

- Change directory to `/src` (this is where all the packages are, can also be put in subfolders):

`cd src`

- Use catkin-tools to create packages. This will create a new package folder in `/src` that has your necessary files. 

`catkin create pkg <package-name>`

`cd <package-name>`

- Rule of thumb: add files to their respective folders, such as `/scripts` for C++ and Python node files, `/launch` for launch files, etc.

- Refer to [this (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) and [this (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) to write your first nodes.

##Running ORB-SLAM2 and OAK-D pipeline scripts:

- Type `rosrun ORB_SLAM2 STEREO_OAKD {PATH TO VOCABULARY FILE} {PATH TO CAMERA YAML CONFIGURAITON FILE}`
  - For example: `rosrun ORB_SLAM2 STEREO_OAKD /home/rami/nr22-software/include/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/rami/nr22-software/include/ORB_SLAM2/Examples/ROS/ORB_SLAM2/OAKD.yaml`



