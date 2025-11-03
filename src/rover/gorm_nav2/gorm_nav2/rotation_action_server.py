import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from example_interfaces.action import Trigger
from rclpy.action import ActionServer
from .turn_on_spot import turn_on_spot

class RotateInPlaceServer(Node):
    def __init__(self):
        super().__init__('rotate_in_place_server')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/wheel_velocities', 10)
        self.action_server = ActionServer(
            self, Trigger, 'rotate_in_place', self.execute_callback)
        self.get_logger().info('RotateInPlaceServer ready.')

    async def execute_callback(self, goal_handle):
        try:
            ang_vel = float(goal_handle.request.message)
        except Exception:
            ang_vel = 0.4
        _, velocities = turn_on_spot(ang_vel)

        msg = Float64MultiArray(data=velocities.tolist())
        self.publisher_.publish(msg)

        goal_handle.succeed()
        result = Trigger.Result()
        result.success = True
        result.message = f'Rotating at {ang_vel} rad/s'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RotateInPlaceServer()
    rclpy.spin(node)
    rclpy.shutdown()
