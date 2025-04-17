#!/usr/bin/env python3

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.publisher_.publish(msg)
        self.get_logger().info('Re-published /map')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MapRepublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()