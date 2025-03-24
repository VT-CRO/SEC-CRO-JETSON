# crobot_controller

This package contains nodes for controlling the robot using the ros2_control framework.

## Nodes
- `crobot_drive_controller`: a node that implements forward kinemamtics for a mecanum chassis. Takes input twist and outputs wheel commands.
- `odometry`: a ndoes that implements odometry to perform position estimation according to encoder feedback. Reads encoder state via joint state interfaces exposed by the crobot system defined in the `crobot_hardware` package. Publishes an odometry topic and broadcasts it to the tf tree.

## Running the Controller

1. First make sure to run `rsp.launch.py` with `use_sim_time=false` and `use_ros2_control=true`. These parameters are set to these by default:
```
ros2 launch crobot_description rsp.launch.py
```

2. Run the following:
```
ros2 launch crobot_controller crobot_controller.launch.py
```

## Configuration

To configure the controller, modify `config/ros2_control.yaml`.

The file `gazebo_ros2_controller_parameters.yaml` also contains configuration for using ros2 control to control a simulated robot.