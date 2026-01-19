
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 node: Compute 2D rigid transform (SE(2)) between an anchored frame (world)
and RTAB-Map's map frame using ArUco markers at known positions.

- Parameters:
  * world_frame (string): name of the anchored frame (default: "world")
  * map_frame (string): name of the RTAB-Map global frame (default: "map")
  * marker_frame_prefix (string): TF prefix used by your ArUco node (default: "aruco_marker_")
  * known_markers (dict): mapping of marker IDs to {x, y} in world frame
  * min_markers (int): minimum number of markers required to compute transform (default: 2)
  * recompute (bool): if true, recompute transform periodically (default: false)
  * timer_period (float): seconds between recompute attempts (default: 1.0)

- Behavior:
  * Collects observed marker positions in map frame via TF lookups.
  * Computes best-fit SE(2) transform (rotation + translation) from world -> map.
  * Publishes a TransformStamped: parent=map, child=world (so tf2 can transform from world to map).
  * Logs RMS residual error to help validate alignment quality.

Author: Jonas-ready example by M365 Copilot
"""

import math
import numpy as np
from typing import Dict, Tuple, List
import json
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
import tf_transformations as tft #sudo apt install ros-$ROSDISTRO-tf-transformations


def compute_se2_svd(points_world: np.ndarray, points_map: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    Compute 2D rigid transform (rotation R, translation t) that maps:
        points_map ≈ R @ points_world + t
    using SVD (Umeyama-style without scaling).

    Args:
        points_world: Nx2 array of (x,y) in world frame.
        points_map: Nx2 array of (x,y) in map frame.

    Returns:
        R: 2x2 rotation matrix
        t: 2x1 translation vector
        rms_err: root-mean-square alignment error
    """
    assert points_world.shape == points_map.shape and points_world.shape[1] == 2
    n = points_world.shape[0]
    if n < 2:
        raise ValueError("Need at least 2 points to estimate SE(2)")

    cw = points_world.mean(axis=0)  # centroid in world
    cm = points_map.mean(axis=0)    # centroid in map

    Wc = points_world - cw
    Mc = points_map - cm

    # 2x2 covariance H = Wc^T * Mc
    H = Wc.T @ Mc

    # SVD: H = U S V^T, rotation R = V U^T with det correction
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    # ensure proper rotation (no reflection)
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T

    t = cm - R @ cw

    # Compute residual RMS error
    aligned = (R @ points_world.T).T + t
    residuals = aligned - points_map
    rms_err = float(np.sqrt((residuals**2).sum() / n))
    return R, t, rms_err


def load_known_markers(self) -> dict:
    # Try dict passed via YAML (if available)
    if self.has_parameter('known_markers'):
        val = self.get_parameter('known_markers').value
        if isinstance(val, dict):
            return val

    # Fallback: JSON string
    if self.has_parameter('known_markers_json'):
        raw = self.get_parameter('known_markers_json').get_parameter_value().string_value
        if raw.strip():
            try:
                obj = json.loads(raw)
                if isinstance(obj, dict):
                    return obj
                else:
                    self.get_logger().warn("known_markers_json parsed but not a dict.")
            except Exception as e:
                self.get_logger().error(f"known_markers_json parse error: {e}")
    return {}


class WorldMapAligner(Node):
    def __init__(self):

        super().__init__(
            'world_map_aligner',
            automatically_declare_parameters_from_overrides=False
        )

        # Declare ONLY scalar types if you want defaults; they are allowed
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('marker_frame_prefix', 'aruco_marker_')
        self.declare_parameter('min_markers', 2)
        self.declare_parameter('recompute', False)
        self.declare_parameter('timer_period', 0.5)
        self.declare_parameter('known_markers_json', '')
        self.declare_parameter('smooth_window', 10)        # last N estimates to average
        self.declare_parameter('show_known_markers', True)

        self.known_markers = load_known_markers(self)


        # Read parameters (auto-declared from YAML if provided)
        
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.marker_prefix = self.get_parameter('marker_frame_prefix').get_parameter_value().string_value
        self.min_markers = int(self.get_parameter('min_markers').get_parameter_value().integer_value)
        self.recompute = bool(self.get_parameter('recompute').get_parameter_value().bool_value)
        self.timer_period = float(self.get_parameter('timer_period').get_parameter_value().double_value)

        self.smooth_window = int(self.get_parameter('smooth_window').get_parameter_value().integer_value)
        self.show_known_markers = bool(self.get_parameter('show_known_markers').get_parameter_value().bool_value)


        # TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_br = StaticTransformBroadcaster(self)

        self._transform_published = False


        # Rolling buffers for smoothing
        self._tx_buf = deque(maxlen=self.smooth_window)
        self._ty_buf = deque(maxlen=self.smooth_window)
        self._yaw_buf = deque(maxlen=self.smooth_window)


        # Periodic compute
        self.timer = self.create_timer(self.timer_period, self.try_compute_and_publish)

        self.pub_marker_array = self.create_publisher(MarkerArray, 'marker_array', 10)

        self.get_logger().info(f"[Aligner] world_frame='{self.world_frame}', map_frame='{self.map_frame}', "
                               f"marker_prefix='{self.marker_prefix}', min_markers={self.min_markers}, "
                               f"recompute={self.recompute}")
        self.get_logger().info(f"[Aligner] Known_markers IDs: {list(self.known_markers.keys())}")

    def lookup_marker_in_map(self, marker_id: str) -> Tuple[float, float]:
        """
        Lookup marker TF: map <- (marker_frame). Returns (x, y) in map frame.
        """
        marker_frame = f"{self.marker_prefix}{marker_id}"
        # self.get_logger().info(f"[Aligner] Looking for {marker_frame} in TF...")
        try:
            # time=0 means "latest"
            t = self.tf_buffer.lookup_transform(target_frame=self.map_frame,
                                                source_frame=marker_frame,
                                                time=rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            return float(x), float(y)
        except Exception as e:
            raise e

    def gather_correspondences(self) -> Tuple[np.ndarray, np.ndarray, List[str]]:
        """
        Build point sets: world_pts (Nx2), map_pts (Nx2), and list of used IDs.
        """
        world_pts = []
        map_pts = []
        used_ids = []

        for mid, pos in self.known_markers.items():
            if not isinstance(pos, dict) or 'x' not in pos or 'y' not in pos:
                self.get_logger().warn(f"Marker {mid} missing 'x'/'y' fields in known_markers.")
                continue

            try:
                mx, my = self.lookup_marker_in_map(mid)
            except Exception:
                # Could not find TF for this marker right now
                continue

            world_pts.append([float(pos['x']), float(pos['y'])])
            map_pts.append([mx, my])
            used_ids.append(str(mid))

        if len(world_pts) == 0:
            return np.empty((0, 2)), np.empty((0, 2)), []
        return np.array(world_pts, dtype=np.float64), np.array(map_pts, dtype=np.float64), used_ids

    def publish_markers(self):
        if not self.show_known_markers:
            return

        marker_array = MarkerArray()
        for mid, pos in self.known_markers.items():
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "known_markers"
            marker.id = int(mid)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(pos['x'])
            marker.pose.position.y = float(pos['y'])
            marker.pose.position.z = 0.25
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.pub_marker_array.publish(marker_array)

    def try_compute_and_publish(self):
        # Stop after publishing once unless recompute=True
        if self._transform_published and not self.recompute:
            return

        self.publish_markers()

        world_pts, map_pts, used_ids = self.gather_correspondences()

        if world_pts.shape[0] < self.min_markers:
            self.get_logger().info(
                f"[Aligner] Observed {world_pts.shape[0]} markers (need >= {self.min_markers}). "
                f"Waiting for TF frames like '{self.marker_prefix}<id>' ..."
            )
            return
        else:
            self.get_logger().info(f"[Aligner] Found {world_pts.shape[0]} markers: IDs {used_ids}")

        try:
            R, t, rms_err = compute_se2_svd(world_pts, map_pts)
        except Exception as e:
            self.get_logger().error(f"[Aligner] Failed computing SE(2): {e}")
            return



        yaw = math.atan2(R[1,0], R[0,0])
        tx, ty = float(t[0]), float(t[1])

        # Push into buffers
        self._tx_buf.append(tx)
        self._ty_buf.append(ty)
        self._yaw_buf.append(yaw)

        # Compute smoothed values
        tx_s = float(np.mean(self._tx_buf))
        ty_s = float(np.mean(self._ty_buf))
        yaw_s = circular_mean(self._yaw_buf)


        # Build TransformStamped: parent=map, child=world
        T = TransformStamped()
        T.header.stamp = self.get_clock().now().to_msg()
        T.header.frame_id = self.map_frame
        T.child_frame_id = self.world_frame

        T.transform.translation.x = tx_s
        T.transform.translation.y = ty_s
        T.transform.translation.z = 0.0

        yaw = math.atan2(R[1, 0], R[0, 0])
        qx, qy, qz, qw = tft.quaternion_from_euler(0.0, 0.0, yaw_s)
        T.transform.rotation.x = qx
        T.transform.rotation.y = qy
        T.transform.rotation.z = qz
        T.transform.rotation.w = qw

        # Publish as static (latched). If recompute=True, we re-broadcast (ok in ROS 2).
        self.static_br.sendTransform(T)
        self._transform_published = True

        # Diagnostics
        self.get_logger().info(
            f"[Aligner] Published TF '{self.map_frame}' <- '{self.world_frame}' using markers {used_ids}.\n"
            f"  Translation t = [{t[0]:.3f}, {t[1]:.3f}] m, Yaw = {math.degrees(yaw):.2f}°\n"
            f"  RMS alignment error = {rms_err:.3f} m (lower is better)"
        )


def circular_mean(yaws_rad) -> float:
    """Compute circular mean of yaw angles (radians)."""
    if len(yaws_rad) == 0:
        return 0.0
    s = sum(math.sin(a) for a in yaws_rad)
    c = sum(math.cos(a) for a in yaws_rad)
    return math.atan2(s, c)


def main(args=None):
    rclpy.init(args=args)
    node = WorldMapAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
