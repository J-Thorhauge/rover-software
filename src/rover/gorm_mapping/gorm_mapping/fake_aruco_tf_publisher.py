
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 node: Publish fake ArUco marker TF frames for testing alignment.

Publishes transforms:
    parent_frame <- aruco_marker_<id>

Parameters (declare via YAML/CLI):
  parent_frame (string): TF parent frame, default "map"
  marker_frame_prefix (string): child frame prefix, default "aruco_marker_"
  marker_ids (list<int or string>): e.g., [1, 2, 7]
  marker_xy   (list<float>): flattened coordinates [x1, y1, x2, y2, ...]
  global_yaw_deg (float): global yaw (deg) applied to all marker positions (simulates map/start misalignment), default 0.0
  position_noise_std (float): Gaussian std-dev in meters added per update, default 0.0
  yaw_noise_std_deg (float): Gaussian std-dev in degrees for per-marker yaw noise, default 0.0
  publish_rate_hz (float): TF publish rate, default 10.0

Author: Jonas test utility by M365 Copilot
"""

import math
import random
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations as tft


def rotate2d(x: float, y: float, yaw_rad: float) -> Tuple[float, float]:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return c * x - s * y, s * x + c * y


class FakeArucoTFPublisher(Node):
    def __init__(self):
        super().__init__('fake_aruco_tf_publisher')

        # Parameters (all scalars/lists to keep it Humble-friendly)
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('marker_frame_prefix', 'aruco_marker_')
        self.declare_parameter('marker_ids', [51, 52, 53, 54, 55, 56])
        self.declare_parameter('marker_xy', [0.5, 3.0, 0.5, 6.0, -1.0, 6.0, -0.5, 9.0, 0.5, 12.0, 0.0, 13.0])
        # self.declare_parameter('marker_ids', [1, 2, 7])
        # self.declare_parameter('marker_xy', [2.0, 0.5, 5.0, 0.5, 2.0, 3.0])
        self.declare_parameter('global_yaw_deg', 0.0)
        self.declare_parameter('position_noise_std', 0.0)
        self.declare_parameter('yaw_noise_std_deg', 0.0)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.marker_prefix = self.get_parameter('marker_frame_prefix').get_parameter_value().string_value
        ids_param = self.get_parameter('marker_ids').value or []
        xy_param = self.get_parameter('marker_xy').value or []
        self.global_yaw_deg = float(self.get_parameter('global_yaw_deg').get_parameter_value().double_value)
        self.pos_noise_std = float(self.get_parameter('position_noise_std').get_parameter_value().double_value)
        self.yaw_noise_std_deg = float(self.get_parameter('yaw_noise_std_deg').get_parameter_value().double_value)
        self.rate_hz = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value)

        # Normalize IDs to strings for consistent child frame naming
        self.marker_ids: List[str] = [str(x) for x in ids_param]
        self.marker_xy: List[float] = [float(v) for v in xy_param]

        if len(self.marker_xy) != 2 * len(self.marker_ids):
            self.get_logger().error(
                f"marker_xy length ({len(self.marker_xy)}) != 2 * len(marker_ids) ({len(self.marker_ids)})"
            )
            raise RuntimeError("Invalid parameters: marker_ids / marker_xy mismatch")

        # Ground-truth positions in "world" before applying global yaw
        self.world_positions: Dict[str, Tuple[float, float]] = {}
        for i, mid in enumerate(self.marker_ids):
            x = self.marker_xy[2 * i]
            y = self.marker_xy[2 * i + 1]
            self.world_positions[mid] = (x, y)

        self.global_yaw_rad = math.radians(self.global_yaw_deg)

        self.br = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self._on_timer)

        self.get_logger().info(
            f"[FakeArucoTF] parent='{self.parent_frame}', prefix='{self.marker_prefix}', "
            f"ids={self.marker_ids}, global_yaw={self.global_yaw_deg:.2f}°, "
            f"pos_noise_std={self.pos_noise_std} m, yaw_noise_std={self.yaw_noise_std_deg}°, "
            f"rate={self.rate_hz} Hz"
        )

    def _on_timer(self):
        now = self.get_clock().now().to_msg()

        for mid, (wx, wy) in self.world_positions.items():
            # Apply global yaw to simulate a rotated 'map' vs 'world'
            x_rot, y_rot = rotate2d(wx, wy, self.global_yaw_rad)

            # Add optional Gaussian noise (position and yaw)
            nx = random.gauss(0.0, self.pos_noise_std) if self.pos_noise_std > 0.0 else 0.0
            ny = random.gauss(0.0, self.pos_noise_std) if self.pos_noise_std > 0.0 else 0.0
            yaw_noise_rad = math.radians(
                random.gauss(0.0, self.yaw_noise_std_deg)
            ) if self.yaw_noise_std_deg > 0.0 else 0.0

            tx = x_rot + nx
            ty = y_rot + ny
            tz = 0.0

            # Give each marker a yaw equal to the global yaw + small noise (purely illustrative)
            yaw = self.global_yaw_rad + yaw_noise_rad
            qx, qy, qz, qw = tft.quaternion_from_euler(0.0, 0.0, yaw)

            T = TransformStamped()
            T.header.stamp = now
            T.header.frame_id = self.parent_frame
            T.child_frame_id = f"{self.marker_prefix}{mid}"
            T.transform.translation.x = tx
            T.transform.translation.y = ty
            T.transform.translation.z = tz
            T.transform.rotation.x = qx
            T.transform.rotation.y = qy
            T.transform.rotation.z = qz
            T.transform.rotation.w = qw

            self.br.sendTransform(T)


def main(args=None):
    rclpy.init(args=args)
    node = FakeArucoTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()