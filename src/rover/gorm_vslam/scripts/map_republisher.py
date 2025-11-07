#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'rtabmap/map',
            self.map_callback,
            10)
        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            10)
        self.get_logger().info('Map republisher started: rtabmap/map → /map')

    def map_callback(self, msg):
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
