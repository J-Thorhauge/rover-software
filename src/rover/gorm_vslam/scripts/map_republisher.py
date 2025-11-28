#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')

        self.last_map = None

        self.subscription = self.create_subscription(
            OccupancyGrid,
            'rtabmap/map',
            self.map_callback,
            10)

        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            10)

        # Timer to periodically republish the last map:
        self.timer = self.create_timer(
            0.5,             # publish at 2 Hz 
            self.timer_callback)

        self.get_logger().info('Map republisher started')

    def map_callback(self, msg):
        self.last_map = msg

    def timer_callback(self):
        if self.last_map is not None:
            # Always publish latest known map
            self.publisher.publish(self.last_map)

def main(args=None):
    rclpy.init(args=args)
    node = MapRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
