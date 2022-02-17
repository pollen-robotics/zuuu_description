## Context
Temporary repository to build the mechanical description of the zuu mobile base.
Eventually, will be merged into :
https://github.com/pollen-robotics/reachy2021_ros2_control/tree/main/reachy_description

## Setup
Add to your bashrc:
```
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
export ZUUU_MAP_NAME=hospital
```

## Usage
```
ros2 launch zuuu_description display.launch.py
```