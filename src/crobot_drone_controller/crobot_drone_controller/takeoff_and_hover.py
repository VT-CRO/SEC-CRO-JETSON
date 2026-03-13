#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from mavros_msgs.msg import State, Thrust
from mavros_msgs.srv import CommandBool, SetMode


class NoGpsLiftHoverLand(Node):
    def __init__(self):
        super().__init__('no_gps_lift_hover_land')

        self.declare_parameter('mode', 'GUIDED_NOGPS')
        self.declare_parameter('rise_time_s', 2.0)
        self.declare_parameter('hover_time_s', 3.0)
        self.declare_parameter('descend_time_s', 2.0)

        # GUID_OPTIONS = 0 interpretation:
        # 0.5 = no climb, >0.5 climb, <0.5 descend
        self.declare_parameter('rise_thrust', 1.00)
        self.declare_parameter('hover_thrust', 0.50)
        self.declare_parameter('descend_thrust', 0.35)

        self.state = State()

        self.create_subscription(State, '/mavros/state', self._state_cb, 10)

        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')

        # MAVROS attitude/thrust topics
        self.att_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_attitude/attitude', 10)
        self.thrust_pub = self.create_publisher(Thrust, '/mavros/setpoint_attitude/thrust', 10)

    def _state_cb(self, msg: State):
        self.state = msg

    def wait_for_service(self, client, name: str):
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {name}...')

    def call_and_wait(self, client, request, label: str, timeout_sec: float = 5.0):
        future = client.call_async(request)
        start = time.time()

        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                self.get_logger().error(f'{label} timed out')
                return None

        resp = future.result()
        if resp is None:
            self.get_logger().error(f'{label} failed: no response')
            return None

        self.get_logger().info(f'{label} response: {resp}')
        return resp

    def set_mode(self, mode_name: str):
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode_name
        return self.call_and_wait(self.set_mode_cli, req, f'Set mode to {mode_name}')

    def arm(self, value: bool = True):
        req = CommandBool.Request()
        req.value = value
        return self.call_and_wait(self.arm_cli, req, 'Arm' if value else 'Disarm')

    def level_quaternion(self) -> Quaternion:
        q = Quaternion()
        q.w = 1.0
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        return q

    def publish_attitude_thrust(self, thrust_value: float):
        att = PoseStamped()
        att.header.stamp = self.get_clock().now().to_msg()
        att.pose.orientation = self.level_quaternion()

        thrust = Thrust()
        thrust.header.stamp = att.header.stamp
        thrust.thrust = float(thrust_value)

        self.att_pub.publish(att)
        self.thrust_pub.publish(thrust)

    def stream_for(self, thrust_value: float, duration_s: float, rate_hz: float = 20.0, label: str = ''):
        self.get_logger().info(f'{label} thrust = {thrust_value:.2f} for {duration_s:.1f}s')
        period = 1.0 / rate_hz
        end_time = time.time() + duration_s

        while rclpy.ok() and time.time() < end_time:
            self.publish_attitude_thrust(thrust_value)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def run(self):
        self.wait_for_service(self.set_mode_cli, 'set_mode')
        self.wait_for_service(self.arm_cli, 'arming')

        self.get_logger().info('Waiting for FCU connection...')
        while rclpy.ok() and not self.state.connected:
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().info('Connected to FCU')

        mode = self.get_parameter('mode').value
        rise_time = float(self.get_parameter('rise_time_s').value)
        hover_time = float(self.get_parameter('hover_time_s').value)
        descend_time = float(self.get_parameter('descend_time_s').value)
        rise_thrust = float(self.get_parameter('rise_thrust').value)
        hover_thrust = float(self.get_parameter('hover_thrust').value)
        descend_thrust = float(self.get_parameter('descend_thrust').value)

        self.stream_for(hover_thrust, 1.5, label='Pre-stream neutral')

        self.set_mode(mode)
        time.sleep(0.5)

        self.arm(True)
        time.sleep(0.5)

        # Keep streaming continuously or GUID_TIMEOUT will stop the behavior.
        self.stream_for(rise_thrust, rise_time, label='Rise')
        self.stream_for(hover_thrust, hover_time, label='Approx hover')
        self.stream_for(descend_thrust, descend_time, label='Descend')
        self.stream_for(0.0, 1.0, label='Throttle cut')

        # Optional disarm attempt after touchdown
        self.arm(False)

        self.get_logger().info('Sequence complete')
        while rclpy.ok():
            self.publish_attitude_thrust(0.0)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)


def main():
    rclpy.init()
    node = NoGpsLiftHoverLand()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()