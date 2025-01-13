# crobot_description

This package contains the URDF files for the crobot. Also includes a mesh and URDF of the game field in `models/meshes`. All external models used should be placed in this folder.

## Configuration

- `config/` contains useful rviz views for viewing the robot and simulated sensor data.
- `worlds/` contains premade environments to simulate in gazebo

## Launch Files
- `rsp.launch.py` - Launches robot state publisher to broadcast the robot description (to be used by Gazebo or to visualize in RVIZ)

## Resources
- Setting up hardware interfaces in robot description: https://github.com/ros-controls/roadmap/blob/master/design_drafts/hardware_access.md