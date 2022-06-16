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

Create a /meshes folder:
```
mkdir -p zuuu_description/meshes
cd zuuu_description/meshes
```

Download the mesh used: [link](https://drive.google.com/file/d/1y3KqgaIK0916n6ELnhmQw_U-jSFYrvL3/view?usp=sharing%29).

## Usage
```
ros2 launch zuuu_description display.launch.py
```
