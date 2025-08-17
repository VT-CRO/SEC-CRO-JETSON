Getting Started
===============

What is this?
#############

This project contains the entire autonomy stack of the CROBOT, a competition robot for IEEE SouthEastCon.
The robot's software architecture is based on a layered architecture with the following layers:

- Application
- Navigation
- Control

Each layer's purpose will be discussed in the following sections.

Application Layer
^^^^^^^^^^^^^^^^^
The application layer is the highest level containing the most abstract instructions for the robot.
It is responsible for planning actions that the robot will take. In the past Finite State Machines (FSMs)
have been used to handle this layer, but behavior trees will take over as the application layer.

BehaviorTree.CPP is the framework that will be used to create and run behavior trees. 
It also is compatible with Groot2 which offers visual tools for debugging and creating behavior trees.

Navigation
^^^^^^^^^^
This layer consists of the path planning, localization, and SLAM technologies. 

nav2 is the path planner used and is a package compatible with ROS.

amcl is used for localization.

rtabmap is used for SLAM

Control
^^^^^^^
This layer is responsible for interfacing with hardware.

ros2 control is used to communicate with a teensy that will be executing hardware commmands.

How to Use
##########
Dependencies should all be handled by the Simulation Environment provided.