Launching the gazebo:
1. Enter the container
2. source the local setup
3. run ros2 launch crobot_gazebo gazebo.launch.py 

you can then make a new terminal and control the wheels by running this command:
ros2 topic pub /ankle_position_controller/commands std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0]}"
where the inputs are front left, front right, rear left, rear right, in radians

and control the wheel velocities with this:
ros2 topic pub /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray \
"{data: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"
front left, mid left, rear left, front right, mid right, rear right