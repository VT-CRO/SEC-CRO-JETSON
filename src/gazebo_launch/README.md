Current Documentation Progress:

Current Progress:
1. added launch folder to the install dependencies in CMakeLists.txt
2. installed ros2 gazebo simulator "sudo apt install ros-<your_ros_distro>-ros-gz-sim"
3. Update CMakeList packages and finished code for launch.py

Launching the gazebo:
1. Enter the container
2. source the local setup
3. run ros2 launch gazebo_launch launch.py

CURRENT ISSUE: this might be a me issue, but my docker container cannot run any sorts of GUI

Note: running this line will try to open an empty gazebo world. This will tell you where issues are
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf

you can then make a new terminal and control the wheels by running this command:
ros2 topic pub /ankle_position_controller/commands std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0]}"
where the inputs are front left, front right, rear left, rear right, in radians

and control the wheel velocities with this:
ros2 topic pub /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray \
"{data: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"
front left, mid left, rear left, front right, mid right, rear right