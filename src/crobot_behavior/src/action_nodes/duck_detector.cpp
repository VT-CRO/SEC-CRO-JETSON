#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class DuckDetectorNode(Node):
    def __init__(self):
        super().__init__('duck_detector')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.pub = self.create_publisher(Bool, '/duck_detected', 10)

        # Tunable params — expose as ROS params for easy field adjustment
        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('yellow_h_low', 20)
        self.declare_parameter('yellow_h_high', 35)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to HSV — much more robust than BGR for color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h_low  = self.get_parameter('yellow_h_low').value
        h_high = self.get_parameter('yellow_h_high').value
        lower_yellow = np.array([h_low,  100, 100])
        upper_yellow = np.array([h_high, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Morphological cleanup to remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        min_area = self.get_parameter('min_contour_area').value
        duck_found = any(cv2.contourArea(c) > min_area for c in contours)

        self.pub.publish(Bool(data=duck_found))

def main(args=None):
    rclpy.init(args=args)
    node = DuckDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()