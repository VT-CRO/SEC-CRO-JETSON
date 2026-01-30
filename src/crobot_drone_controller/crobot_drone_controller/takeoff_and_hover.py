#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

class TakeoffAndHover(Node):
    def __init__(self):
        super().__init__('takeoff_and_hover')

        self.declare_parameter('takeoff_altitude', 1.5)
        self.declare_parameter('mode', 'GUIDED')
        self.declare_parameter('connect_timeout_s', 15.0)

        self.state = State()

        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

    def _state_cb(self, msg):
        self.state = msg
    def wait_for(self, client, name):
        while not client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info(f"Waiting for {name}... ")

    def run(self):
        self.wait_for(self.set_mode_cli, "set_mode")
        self.wait_for(self.arm_cli, "arming")
        self.wait_for(self.takeoff_cli, "takeoff")

        self.get_logger().info("Waiting for FCU connection...")
        while rclpy.ok() and not self.state.connected:
            rclpy.spin_once(self, timeout_sec = 0.2)

        self.get_logger().info("Connected.")

        # guided mode
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = self.get_parameter('mode').value
        self.set_mode_cli.call_async(req)
        time.sleep(2)

        # arm
        req = CommandBool.Request()
        req.value = True
        self.arm_cli.call_async(req)
        time.sleep(2)

        # takeoff
        req = CommandTOL.Request()
        req.altitude = float(self.get_parameter('takeoff_altitude').value)
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0
        self.takeoff_cli.call_async(req)

        self.get_logger().info("Takeoff command sent.")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec = 1.0)
            

def main():
    rclpy.init()
    node = TakeoffAndHover()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()