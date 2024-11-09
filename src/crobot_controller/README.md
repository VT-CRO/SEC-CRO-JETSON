# crobot_controller

This package contains nodes for controlling the robot using the ros2_control framework.

## Nodes
- `crobot_controller`: a node that implements forward kinemamtics for a mecanum chassis. Takes input twist and outputs wheel commands.
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

To configure the controller, modify `config/ros2_control.yaml`. Configuration parameters include:

- front_left_wheel_name: a string specifying the name of the front left wheel joint
- back_left_wheel_name: a string specifying the name of the back left wheel joint
- front_right_wheel_name: a string specifing the name of the front right wheel joint
- back_right_wheel_name: a string specifing the name of the back right wheel joint
- open_loop: a boolean that calculates the robot odometry from input commands instead of encoder feedback if set to true
- position_feedback: a boolean that calculates robot odometry from wheel positions instead of velocity if set to true
- wheel_separation: a double that indicates the separation of wheels in meters
- wheel_separation_multiplier: a multiplier to scale the wheel separation
- wheel_radius: a double that indicates the radius of each wheel in meters
- back_left_wheel_radius_multiplier: a multiplier for scaling the radius of the back left wheel
- back_right_wheel_radius_multiplier: a multiplier for scaling the radius of the back right wheel
- front_left_wheel_radius_multiplier: a multiplier for scaling the radius of the front left wheel
- front_right_wheel_radius_multiplier: a multiplier for scaling the radius of the front right wheel
- tf_frame_prefix_enable: a boolean that enables appending tf_frame_prefix to tf frame ids published by the controller node
- tf_frame_prefix: a string specifying the tf frame to be appended to tf frames published by the controller node if enabled. If this parameter is empty, the controller's namespace will be used instead.
- odom_frame_id: a string specifying the name of the tf id to be used for the odometry frame
- base_frame_id: a string specifying the id of the robot's base frame that is a child of the odometry frame
- pose_covariance_diagonal: an array of doubles containing the odometry covariance for encoder output of the robot for the pose. It is recommend to tune these values based on the robot's sample odometry data
- twist_covariance_diagonal: an array of doubles containing the odometry covariance for the encoder output of the robot for the speed. It is recommended to tune these values based on the robot's sample odometry data
- enable_odom_tf: a boolean that enables publishing of the transform between `odom_frame_id` and `base_frame_id`
- cmd_vel_timeout: a double indicating how much time to wait in seconds before considering the the input command on `cmd_vel` topic to be stale
publish_limited_velocity: a boolean that enables publishing of the `cmd_vel` input with linear and angular limits applied
- velocity_rolling_window_size: an integer indicating the size of the rolling window for calculation of mean velocity use in odometry
publish_rate: a double indicating the rate at which to publish the odometry and tf messages.",
- linear: a json object that specifies the limits to apply to the linear component of the `cmd_vel` input. See `src/crobot_controller_parameters.yaml` for more info
- angular: a json object that specifies the limits to apply to the angular component of the `cmd_vel` input. See `src/crobot_controller_parameters.yaml` for more info.

The file `gazebo_ros2_controller_parameters.yaml` also contains configuration for using ros2 control to control a simulated robot.