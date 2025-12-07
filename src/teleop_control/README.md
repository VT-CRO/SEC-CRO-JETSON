## Usage

### 1. Build the Package

### 2. Launch the Mapper Node
This node will listen for commands and translate them to the robot controllers.
```bash
ros2 launch teleop_control teleop_launch.py
```

### 3. Run Teleop Keyboard
In a separate terminal, run the standard ROS 2 teleop node to generate commands:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
