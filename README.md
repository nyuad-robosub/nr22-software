# nr22-software
Repository for Robosub 2022 software team.

The repository is structured as a catkin workspace with only the `/src` directory (ignoring all of the catkin files and folders).

## Installing

- Clone the nr22-software and cd into it: (clone it outside of pre-existing workspaces, if any)

`git clone https://github.com/nyuad-robosub/nr22-software` (add `-b <branch>` if cloning a specific branch, and add `--single-branch` if only want to fetch single branch)

`cd nr22-software`

- For first time cloning, init all the submodules:

`git submodule update --init --recursive`

- To update submodules to latest version:

`git submodule update --recursive --remote`

- If there are no pre-existing catkin workspaces (only ROS exists on the system): extend `nr22-software` to ROS Melodic install folder
  - `catkin config --init --extend /opt/ros/melodic`

- If there are already a workspace (such as `~/gz_ws`) the extend above need to be extending to that instead:
  - `catkin config --init --extend <path-to-last-workspace>`
  - Example: `catkin config --init --extend ~/gz_ws`

- Build the packages inside (this command can be run anywhere inside the `nr22-software` folder):

`catkin build` (to build specific packages use `catkin build <package-name>`)

- Source the new workspace after building (this needs to be done everytime a new package is built, and everytime a terminal is opened, for ROS to recognize it):

`source devel/setup.bash`

- If you don't want to do that everytime, do this:

`echo 'source <path-to-folder-containing-nr22-software>/nr22-software/devel/setup.bash' > ~/.bashrc` (which will write the source command to `~/.bashrc` and each new terminal will run it)

## Creating new packages

- References: https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html

- Change directory to `/src`:

`cd src`

- Use catkin-tools to create packages. This will create a new package folder in `/src` that has your necessary files. 

`catkin create pkg <package-name>`

`cd <package-name>`

- Rule of thumb: add files to their respective folders, such as `/scripts` for C++ and Python node files, `/launch` for launch files, etc.

- Refer to [this (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) and [this (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) to write your first nodes.
