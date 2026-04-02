# Teleop Instructions

1. Get the IP address of the jetson. This can be done by connecting it to a monitor, opening a terminal and running ```hostname -I```. The IP will be the first string of numbers, e.g. `172.16.96.205`

2. Keep record of the IP, and then you can disconnect the jetson from the monitor.

3. Open three terminals on your laptop and ssh into the jetson in each terminal using `ssh vtcro@172.16.96.205`, but replace `172.16.96.205` with the actual IP address. You should know what the password is.

4. In one terminal, run bringup
```
ros2 launch crobot_bringup bringup_testing.launch.py
```

5. In a second terminal, run the teleop node
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Instructions of what keys to use for teleop should show up in your terminal. 

6. Turn the speeds way down (around 0.10 to 0.15)

7. You should now be good to move the robot using teleop.

8. To control other peripherals, you need to publish messages to their corresponding topics. 

9. In the third terminal, adjust the following commands based on what you want to move.

To control the sweeper, use the following and adjust the data value in radians. 3.0 will move the sweeper all the way up.
```
ros2 topic pub --once /sweeper_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [3.0]}"
```

To control the winch, use the command below, and change the data value to whatever velocity you want.
```
ros2 topic pub --once /winch_velocity_controller/commands std_msgs/msg/Float64 "{data: 0.35}"
```

I don't recommend using this right now, but you can initiate the crater run sequence with the following.
```
ros2 topic pub --once /crobot_embedded_mode std_msgs/msg/String "data: craterRun"
```

If you use that, you will need to go back to write mode using
```
ros2 topic pub --once /crobot_embedded_mode std_msgs/msg/String "data: write"
```
