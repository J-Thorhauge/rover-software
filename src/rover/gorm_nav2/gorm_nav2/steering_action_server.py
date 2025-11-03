import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from example_interfaces.action import Trigger
from rclpy.action import ActionServer
from .turn_on_spot import turn_on_spot
import numpy as np


class SetSteeringServer(Node):
    def __init__(self):
        super().__init__('set_steering_server')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/steering_angles', 10)
        self.action_server = ActionServer(
            self, Trigger, 'set_steering', self.execute_callback)
        self.get_logger().info('SetSteeringServer ready.')

    async def execute_callback(self, goal_handle):
        target = goal_handle.request.message.strip().lower()
        msg = Float64MultiArray()

        if target == 'turn_on_spot':
            angles, _ = turn_on_spot(0.0)
        elif target == 'straight':
            angles = np.zeros(4)
        else:
            self.get_logger().warn(f'Unknown steering target: {target}')
            angles = np.zeros(4)

        msg.data = angles.tolist()
        self.publisher_.publish(msg)

        goal_handle.succeed()
        result = Trigger.Result()
        result.success = True
        result.message = f'Set steering to {target}'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SetSteeringServer()
    rclpy.spin(node)
    rclpy.shutdown()
