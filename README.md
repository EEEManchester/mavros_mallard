A custom MAVROS for MALLARD  

## Installation  
1. Install catkin_tool and dependencies:  
```
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```  
> For Noetic use that:  
> ```
> sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y
> ```

2. Create the workspace: unneeded if you already has workspace

```
mkdir -p ~/catkin_ws/src  
cd ~/catkin_ws  
catkin init  
wstool init src  
```
3. Add MAVLink package discriptions

we use the Melodic reference for all ROS distros as it's not distro-specific and up to date
```
rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall
```

5. Merge package discriptions
```
wstool merge -t src /tmp/mavros.rosinstall
```

6. Add our custom mavros package discriptions. Open `catkin_ws/src/.rosinstall` with your desired text editor, e.g.:
```
gedit ~/catkin_ws/src/.rosinstall
```
Add the following lines at the end of the file
```
- git:  
    local-name: mavros_mallard  
    uri: https://github.com/EEEManchester/mavros_mallard.git  
    version: dev'  
```    
7. The install both packages
```
wstool update -t src -j4`  
rosdep install --from-paths src --ignore-src -y
```

8. Install GeographicLib datasets:  
```
sudo ./src/mavros_mallard/mavros/scripts/install_geographiclib_datasets.sh
```

9. Build   
```
catkin build
```

10. Make sure that you use setup.bash or setup.zsh from workspace. Else rosrun can't find nodes from this workspace.
```
source devel/setup.bash
```

### Original guides
1. [README](https://github.com/EEEManchester/mavros_mallard/blob/master/README_MAVROS.md)
2. [Installation instructions](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)
