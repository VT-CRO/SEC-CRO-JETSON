# crobot_bringup

This package contains launch files for launching the complete autonomous robot application in either simulation or on real hardware.

## Launch Files

- `bringup_simulated.launch.py` - Launches a fully simulated robot in Gazebo with navigation
- `bringup.launch.py` - Launches autonomous robot application with real hardware

## How To Use

[TODO: Add ability to switch between mapping mode and autonomous mode]

## Note

As of now, we're having an issue where april tags aren't being detected in the simulation due to lighting issues. A temporary fix would be to just create a point light (see circled in the image below) and positioning it somewhere above the field.