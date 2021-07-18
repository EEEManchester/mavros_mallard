# mavros_mallard
A custom MAVROS for MALLARD
### Source installation

Use `wstool` utility for retrieving sources and  [`catkin` tool](https://catkin-tools.readthedocs.io/en/latest/)for build.

NOTE: The source installation instructions are for the ROS Melodic release.

```sh
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
# For Noetic use that:
# sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y

# 1. Create the workspace: unneeded if you already has workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src

# 2. Install MAVLink
#    we use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall

# 3. Install MAVROS: get source (upstream - released)
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
# alternative: latest source
# rosinstall_generator --upstream-development mavros | tee -a /tmp/mavros.rosinstall
# For fetching all the dependencies into your catkin_ws, just add '--deps' to the above scripts
# ex: rosinstall_generator --upstream mavros --deps | tee -a /tmp/mavros.rosinstall

# 4. Create workspace & deps
wstool merge -t src /tmp/mavros.rosinstall
gedit ~/catkin_ws/src/.rosinstall
change 
'- git:
    local-name: mavros
    uri: https://github.com/mavlink/mavros.git
    version: 1.8.0'
to
'- git:
    local-name: mavros_mallard
    uri: https://github.com/EEEManchester/mavros_mallard.git
    version: dev'
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y

# 5. Install GeographicLib datasets:
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# 6. Build source
catkin build

# 7. Make sure that you use setup.bash or setup.zsh from workspace.
#    Else rosrun can't find nodes from this workspace.
source devel/setup.bash
```
