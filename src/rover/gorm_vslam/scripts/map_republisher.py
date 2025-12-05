#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')
        self.last_msg_time = None
        self.last_map = None
        self.timer = self.create_timer(1.0, self.timer_callback)

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

        self.last_map = msg
        self.last_msg_time = self.get_clock().now()
        self.publisher.publish(msg)

    def timer_callback(self):
       
        self.time_now = self.get_clock().now()
        if self.last_msg_time is not None:
            
            dt = (self.time_now-self.last_msg_time).nanoseconds/1e9
            if dt >2.0:
                self.get_logger().info('No map received from RTAB-map: republishing last map saved')
                
                self.publisher.publish(self.last_map)




def main(args=None):
    rclpy.init(args=args)
    node = MapRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
