#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import csv

class CostmapExporter(Node):
    def __init__(self):
        super().__init__('costmap_exporter')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))

        # Save to CSV
        with open('costmap_export.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(data)

        self.get_logger().info('✅ Costmap exported to costmap_export.csv')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CostmapExporter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
