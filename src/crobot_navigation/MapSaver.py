import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import yaml
import numpy as np
import sys
from PIL import Image

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/nvblox_node/static_occupancy_grid',
            self.map_callback,
            qos
        )
        self.get_logger().info('Waiting for map...')

    def map_callback(self, msg):
        width, height = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        img = np.zeros((height, width), dtype=np.uint8)
        img[data == 0] = 254
        img[data == -1] = 205
        img[data == 100] = 0
        Image.fromarray(img).save('/ssd/olivia_test_ws/SEC-NEW-JETSON/map.pgm')
        with open('/ssd/olivia_test_ws/SEC-NEW-JETSON/map.yaml', 'w') as f:
            yaml.dump({
                'image': 'map.pgm',
                'resolution': msg.info.resolution,
                'origin': [msg.info.origin.position.x, msg.info.origin.position.y, 0.0],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.25,
            }, f)
        self.get_logger().info('Map saved!')
        rclpy.shutdown()

rclpy.init(args=sys.argv)
node = MapSaver()
rclpy.spin(node)