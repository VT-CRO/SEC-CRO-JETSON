# crobot_hardware

This package defines the interface between crobot_controller and the real hardware.

The hardware interface defined here is designed to communicate with the SEC-CRO firmware (see SEC-CRO-LIB).
 
## Nodes
`crobot_system.cpp`: defines the hardware interface between `crobot_controller` and actuator hardware.

## How To Use

If you wish to use this hardware interfaces with any of your controllers, make sure you do the following:

1. Create a xacro to import the hardware interface into the robot description. This information is used by any nodes that subscribe to the robot description published by `rsp.launch.py`. A sample file is contained in `crobot_description/description/ros2_control.xacro`.

2. Modify the parameters for your hardware (See Configuration).

3. Modify this system to expose the correct state and command interfaces for your requirements. By default the following state/command interfaces are exposed for the following joints:
    - front_left_wheel
        - command: velocity
        - state: velocity, position
    - front_right_wheel
        - command: velocity
        - state: velocity, position
    - back_left_wheel
        - command: velocity
        - state: velocity, position
    - back_right_wheel
        - command: velocity
        - state: velocity, position

Now the hardware system is ready to use by launching robot_state publisher using the provided launch script in crobot_description:

```
ros2 launch crobot_description rsp.launch.py
```

Controllers subscribed to the published robot description will now be able to access the state and command interfaces exposed by this hardware system.

On startup, the interface will establish a serial connection with the microcontroller that controls the actuators. An error message will be output if no microcontroller is connected.

## Configuration

To configure the hardware system for your hardware, modify the parameters in your xacro file. The hardware interface expects the following parameters:

- front_left_wheel_name: a string specifying the front left wheel joint name
- front_right_wheel_name: a string specifying the front right wheel joint name
- back_left_wheel_name: a string specifying the back left wheel joint name
- back_right_wheel_name: a string specifying the back right wheel joint name 
- loop_rate: [not sure what it does]
- dev: a string specifying the device port of the hardware microcontroller
- baud_rate: an integer specifying the baud rate with which to communicate with the microcontroller
- timeout_ms: an integer specifying the amount of time in ms to wait to connect to microcontroller before returning a communication error
- enc_counts_per_rev: an integer specifying the number of encoder counts for 1 motor revolution