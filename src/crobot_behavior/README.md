crobot_behaviorTree
Behavior Tree for Crobot

Launching the Behavior Tree
Launch "behavior.launch.py" in crobot_bringup. This will start the behavior tree ''' ros2 launch src/crobot_bringup/launch/bringup.launch.py '''

TESTING BRANCH WITH TURTLE
1. Export the turtlebot model
export TURTLEBOT3_MODEL=burger

2. Launch the Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

3. Launch RViz
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py   use_sim_time:=True   params_file:=/home/ros/ws/SEC-NEW-JETSON/src/crobot_behavior/config/nav2_params.yaml

4. Launch the behavior tree
ros2 launch src/crobot_bringup/launch/bringup.launch.py 
