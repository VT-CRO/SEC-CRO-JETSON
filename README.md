# Crobot

This workspace contains packages for the autonomous robot application of the VTCRO southeast con team.


This workspace is designed to work with the following 3rd party packages:
- rtabmap: A package to perform SLAM using a depth camera.
- nav2: A set of packages for path planning and trajectory control for autonomous vehicles within a mapped out environment
- behaviortree_ros2: A ros2 wrapper for Behavior Tree, a framework for implementing task planning and execution of autonomous agents

The following packages were made to support the application:
- crobot_description: Contains robot URDFs and other useful configuration for running simulation and controllers
- crobot_gazebo: Contains launch files for launching a simulated robot
- crobot_controller: Defines a controller to translate user/task inputs to actuator commands
- crobot_hardware: Defines the interface between the crobot_controller and actuator hardware
- crobot_navigation: Contains autonomous behaviors and routines for Behavior Tree
- crobot_bringup: Contains a set of centralized launch files that spawn all the necessary packages for a full autonomous robot in either simulation or using real hardware

## Hardware Requirements
- Intel Realsense Camera
- A Microcontroller flashed with the correct SEC-CRO firmware (see SEC-CRO-LIB)

## Getting Started

### Setup Environment

This workspace is designed to work with ROS Humble on Ubuntu.
You may also run the provided ROS Simulation Environment using Docker.
Clone this repository into your desired working directory.

### Install Dependencies
Navigate to the workspace folder and run the following commands
```
sudo apt update
source install/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Building 
The first time you build this repo, build the behaviour tree packages first

```
colcon build --packages-select btcpp_ros2_interfaces behaviortree_ros2

source install/setup.bash

colcon build
```

Whenever you change something in the package you must rebuild and source.
Make sure you are in the ws folder before running these commands 

```
colcon build

source install/setup.bash 
```

### How to Run
To run a simulated test environment run 

```
ros2 launch crobot_bringup bringup_simulated.launch.py
```

To run the stack with your real hardware run
```
ros2 launch crobot_bringup bringup.launch.py
```

See the individual package README's for additional information on configuration.

# Trouble shooting

if the april tags folder is empty try these commands

```
git submodule init

git submodule update
```
## Resources

- Nav2 Docs: https://docs.nav2.org/
- rtabmap Docs: http://wiki.ros.org/rtabmap_ros (These docs are a bit outdated, but give a good idea of how rtabmap fits into everything)
- ros2_control Docs: https://control.ros.org/rolling/index.html
- behaviortree Docs: https://www.behaviortree.dev/docs/intro
- integrating ROS2 with behaviortree: https://www.behaviortree.dev/docs/ros2_integration/
- behaviortree wrapper for ROS2: https://github.com/BehaviorTree/BehaviorTree.ROS2
