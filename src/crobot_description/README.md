# crobot_description

This package contains the URDF files for the crobot.

## Configuration

- `config/` contains useful rviz views for viewing the robot and simulated sensor data.
- `worlds/` contains premade environments to simulate in gazebo

## Launch Files
- `rsp.launch.py` - Launches robot state publisher to broadcast the robot description (to be used by Gazebo or to visualize in RVIZ)

## How To Use

The given `rsp.launch.py` launch allows for using the description in a Gazebo simulation or simply to visualize in RVIZ. When using the published robot state in simulation, please be sure to
run the launch file with the `use_sim_time` paramater set to true. For example:
```
ros2 launch crobot_description use_sim_time:=false
```
Otherwise this paramater should be set to false.

If using ROS2 control, the description also contains hardware interfaces for controlling either real robot hardware (see crobot_hardware) or gazebo. To use these harddwdare interfaces, please be
sure to run `rsp.launch.py` with `use_ros2_control` set to true:
```
ros2 launch crobot_description use_ros2_control:=true
```
If `use_sim_time` is also set to true, then the gazebo hardware interface will be used. Otherwise, the hardware interface defined in crobot_hardware will be enabled.

### Configuring the Camera

The robot description contains a simulated RGB and RGBD camera that can be swapped out. To swap out cameras, in the `description/robot.urdf.xacro` file, find the line and where either `camera.xacro` or `depth_camera.xacro` is included and edit to include the correct camera file.

To modify the camera position and orientation, navigate to the corresponding camera xacro file and find the `camera_joint` it should look like:

[TODO: ADD PICTURE]

To change the position of the camera relative to the robots center, simply edit the `xyz` parameter in the origin tag. Modifying the `rpy` parameter changes the orientation. For reference, the forward facing orientation of the camera with respect to the robot's center is `"0 0 0"` and positive angles in the y-axis pitch the camera down. When modifying this file also be sure not to change the orientation of the camera_optical joint from its default orientation of `"${-pi/2} 0 ${-pi/2}"`. This angle has been set so that the camera image is not inverted, so only change this orientation if you know what you're doing.
