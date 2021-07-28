A custom MAVROS for MALLARD  

## Installation  
### 1. Install catkin_tool and dependencies
For Melodic:
```
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```  
For Noetic:
```
sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y
```

### 2. Create a workspace
```
mkdir -p ~/catkin_ws/src  
cd ~/catkin_ws  
catkin init  
wstool init src  
```

### 3. Add MAVLink package reference
For **both** Melodic and Noetic, we use mavlink melodic reference as it's not distro-specific and up to date.
```
rosinstall_generator --rosdistro melodic mavlink | tee src/.rosinstall
```

### 4. Add our custom mavros package reference
Open `src/.rosinstall` with your desired text editor, e.g.:
```
gedit src/.rosinstall
```
Add the following lines at the end of the file.
```
- git:  
    local-name: mavros_mallard  
    uri: https://github.com/EEEManchester/mavros_mallard.git  
    version: dev
```    

### 5. Install both packages
```
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
```

### 6. Install GeographicLib datasets
```
sudo ./src/mavros_mallard/mavros/scripts/install_geographiclib_datasets.sh
```

### 7. Build and source
```
catkin build
source devel/setup.bash
```

### 8. Add source to bashrc
Optional, add source to bashrc to avoid manual source everytime
```
echo "source $HOME/devel/setup.bash" >> ~/.bashrc 
```

### Original guides
1. [README](https://github.com/EEEManchester/mavros_mallard/blob/master/README_MAVROS.md)
2. [Installation instructions](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)
