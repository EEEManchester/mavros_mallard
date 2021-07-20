A custom MAVROS for MALLARD  
## Source installation  

Use `wstool` utility for retrieving sources and  [catkin tools](https://catkin-tools.readthedocs.io/en/latest/) for build.


NOTE: The source installation instructions are for the ROS Melodic release.

1. Install catkin_tool and dependencies:  
`sudo apt-get install python-catkin-tools python-rosinstall-generator -y`  
2. For Noetic use that:  
`sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y`

3. Create the workspace: unneeded if you already has workspace

```
mkdir -p ~/catkin_ws/src  
cd ~/catkin_ws  
catkin init  
wstool init src  
```
4. Install MAVLink

we use the Melodic reference for all ROS distros as it's not distro-specific and up to date
`rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall`

5. Install MAVROS: get source (upstream - released)
`rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall`

6. Create workspace & deps

    `wstool merge -t src /tmp/mavros.rosinstall`  
`gedit ~/catkin_ws/src/.rosinstall`  
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
`wstool update -t src -j4`  
`rosdep install --from-paths src --ignore-src -y`

7. Install GeographicLib datasets:  
`sudo ./src/mavros_mallard/mavros/scripts/install_geographiclib_datasets.sh`

8. Build source   
`catkin build`

9. Make sure that you use setup.bash or setup.zsh from workspace. Else rosrun can't find nodes from this workspace.
`source devel/setup.bash`

### Original guides
1. [README](https://github.com/EEEManchester/mavros_mallard/blob/master/README_MAVROS.md)
2. [Installation instructions](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)
