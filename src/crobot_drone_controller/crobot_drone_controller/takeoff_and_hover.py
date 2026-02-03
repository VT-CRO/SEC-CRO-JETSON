#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State # mavros_msgs/State is the ROS message that MAVROS publishes

# TOL = takeoff and land:o
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

class TakeoffAndHover(Node):
    def __init__(self):
        # creates the ros node with the name 'takeoff_and_hover'
        super().__init__('takeoff_and_hover')


        # you can change these parameters when you run the node
        # e.g. ros2 run ... --ros-args -p takeoff_altitude:=2.0 -p mode:=GUIDED_NOGPS
        self.declare_parameter('takeoff_altitude', 1.5) # altitude is in meters
        self.declare_parameter('mode', 'GUIDED_NOGPS') # we don't have gps
        self.declare_parameter('connect_timeout_s', 15.0) # after 15 seconds, we will stop waiting to connect

        # check state stuff: https://docs.ros.org/en/api/mavros_msgs/html/msg/State.html
        self.state = State()

        # subscriber
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)

        # calling mavros to change flight mode
        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')

        # calling mavros to arm or disarm so that the flight controller can use tje motors
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')

        # calls mavros, to command that we take off, given an altitude
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
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = self.get_parameter('mode').value
        self.set_mode_cli.call_async(request)
        time.sleep(2)

        # arm
        request = CommandBool.Request()
        request.value = True
        self.arm_cli.call_async(request)
        time.sleep(2)

        # takeoff
        request = CommandTOL.Request()
        request.altitude = float(self.get_parameter('takeoff_altitude').value)
        request.latitude = 0.0
        request.longitude = 0.0
        request.min_pitch = 0.0
        request.yaw = 0.0
        self.takeoff_cli.call_async(request)

        self.get_logger().info("Takeoff command sent.")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec = 1.0)
            

def main():
    # ros!
    rclpy.init()

    # create, run, delete node
    node = TakeoffAndHover()
    node.run()
    node.destroy_node()

    # byebye, ros :(
    rclpy.shutdown()

if __name__ == '__main__':
    main()