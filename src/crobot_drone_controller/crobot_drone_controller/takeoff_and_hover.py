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
        self.declare_parameter('hover_time_s', 5.0)

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

        # command to land
        self.land_cli = self.create_client(CommandTOL, '/mavros/cmd/land')

    def _state_cb(self, msg):
        self.state = msg

    def wait_for_service(self, client, name):
        while rclpy.ok() and not client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info(f"Waiting for {name} service...")

    def call_and_wait(self, client, request, label, timeout_sec = 5.0):
        future = client.call_async(request)
        start = time.time()

        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec = 0.1)

            if (time.time() - start > timeout_sec):
                self.get_logger().error(f"{label} timed out")
                return None
        
        if future.result() is None:
            self.get_logger().error(f"{label} [failed]: no response")
            return None
        
        self.get_logger().info(f"{label} response received")
        return future.result()


    def set_mode(self, mode_name):
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode_name
        return self.call_and_wait(self.set_mode_cli, request, f"Set mode to {mode_name}")


    def arm(self, value = True):
        request = CommandBool.Request()
        request.value = value

        if (value):
            return self.call_and_wait(self.arm_cli, request, "Arm")
        else:
            return self.call_and_wait(self.arm_cli, request, "Disarm")

    def takeoff(self, altitude):
        request = CommandTOL.Request()
        request.altitude = float(altitude)
        request.latitude = 0.0
        request.longitude = 0.0
        request.min_pitch = 0.0
        request.yaw = 0.0
        return self.call_and_wait(self.takeoff_cli, request, f"Takeoff to {altitude} meters")

    def land(self):
        return self.set_mode("LAND")

    def wait_for(self, client, name):
        while rclpy.ok() and not client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info(f"Waiting for {name}... ")

    def run(self):
        self.wait_for(self.set_mode_cli, "set_mode")
        self.wait_for(self.arm_cli, "arming")
        self.wait_for(self.takeoff_cli, "takeoff")
        self.wait_for(self.land_cli, "land")

        self.get_logger().info("Waiting for FCU connection...")
        while rclpy.ok() and not self.state.connected:
            rclpy.spin_once(self, timeout_sec = 0.2)

        self.get_logger().info("Connected.")

        # # guided mode
        # request = SetMode.Request()
        # request.base_mode = 0
        # request.custom_mode = self.get_parameter('mode').value
        # self.set_mode_cli.call_async(request)
        # time.sleep(2)

        # # arm
        # request = CommandBool.Request()
        # request.value = True
        # self.arm_cli.call_async(request)
        # time.sleep(2)

        # # takeoff
        # request = CommandTOL.Request()
        # request.altitude = float(self.get_parameter('takeoff_altitude').value)
        # request.latitude = 0.0
        # request.longitude = 0.0
        # request.min_pitch = 0.0
        # request.yaw = 0.0
        # self.takeoff_cli.call_async(request)

        # self.get_logger().info("Takeoff command sent.")

        mode = self.get_parameter('mode').value
        altitude = self.get_parameter('takeoff_altitude').value
        hover_time = self.get_parameter('hover_time_s').value

        self.set_mode(mode)
        time.sleep(1.0)

        self.arm(True)
        time.sleep(1.0)

        self.takeoff(altitude)
        self.get_logger().info(f"Hovering for {hover_time} seconds...")
        time.sleep(float(hover_time))

        self.get_logger().info("Sending landing command...")
        self.land()

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