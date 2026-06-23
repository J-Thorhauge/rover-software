
#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# ROS 2 Humble ArUco -> TF publisher (supports sensor_msgs/CompressedImage)
#
# Subscribes:
#   - sensor_msgs/CompressedImage (image_topic) when use_compressed=true
#   - sensor_msgs/Image (image_topic) when use_compressed=false
#   - sensor_msgs/CameraInfo (camera_info_topic)
# Publishes:
#   - TF transforms: camera_frame -> aruco_<id>
# Optional:
#   - sensor_msgs/Image visualization (if publish_debug_image=true)

import math
import numpy as np
import cv2
from cv2 import aruco
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ArucoTfNode(Node):
    def __init__(self):
        super().__init__('aruco_tf_node')
        self.bridge = CvBridge()

        # --- Parameters ---
        self.declare_parameter('image_topic', '/zed_front/zed/rgb/image_rect_color/compressed')
        self.declare_parameter('camera_info_topic', '/zed_front/zed/rgb/camera_info')
        self.declare_parameter('marker_length', 0.15)   # meters
        self.declare_parameter('dictionary', 'DICT_5X5_100')
        self.declare_parameter('tf_prefix', '')
        self.declare_parameter('child_frame_prefix', 'aruco_marker_')
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_topic', 'aruco/debug_image')
        self.declare_parameter('use_image_header_stamp', True)
        self.declare_parameter('use_compressed', True)


        # Fetch params
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.marker_length = float(self.get_parameter('marker_length').value)
        self.dictionary_name = self.get_parameter('dictionary').get_parameter_value().string_value
        self.tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value
        self.child_frame_prefix = self.get_parameter('child_frame_prefix').get_parameter_value().string_value
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.debug_image_topic = self.get_parameter('debug_image_topic').get_parameter_value().string_value
        self.use_image_header_stamp = bool(self.get_parameter('use_image_header_stamp').value)
        self.use_compressed = bool(self.get_parameter('use_compressed').value)


        self.R_co = np.array([[1., 0., 0.],
                              [0., 1., 0.],
                              [0., 0., 1.]], dtype=np.float64)


        # --- Prepare ArUco dictionary ---
        self.aruco_dict = self._get_dictionary(self.dictionary_name)
        self.aruco_params = aruco.DetectorParameters_create()

        # --- Camera intrinsics buffer ---
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame_id = None

        # --- TF broadcaster ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Subscribers ---
        if self.use_compressed:
            self.image_sub = self.create_subscription(CompressedImage, self.image_topic, self.compressed_image_callback, 10)
            self.get_logger().info(f"Subscribing to CompressedImage: {self.image_topic}")
        else:
            self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
            self.get_logger().info(f"Subscribing to Image: {self.image_topic}")

        self.caminfo_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, qos_profile_sensor_data)

        # --- Optional debug publisher ---
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10) if self.publish_debug_image else None

        self.get_logger().info(f"CameraInfo topic: {self.camera_info_topic}")
        self.get_logger().info(f"ArUco dict={self.dictionary_name}, marker_length={self.marker_length} m")

    def _get_dictionary(self, name: str):
        mapping = {
            'DICT_4X4_50': aruco.DICT_4X4_50,
            'DICT_4X4_100': aruco.DICT_4X4_100,
            'DICT_5X5_50': aruco.DICT_5X5_50,
            'DICT_5X5_100': aruco.DICT_5X5_100,
            'DICT_6X6_50': aruco.DICT_6X6_50,
            'DICT_6X6_100': aruco.DICT_6X6_100,
            'DICT_7X7_50': aruco.DICT_7X7_50,
            'DICT_7X7_100': aruco.DICT_7X7_100,
            'DICT_APRILTAG_36h11': aruco.DICT_APRILTAG_36h11,  # OpenCV >=4.7
        }
        if name not in mapping:
            self.get_logger().warn(f"Unknown dictionary '{name}', defaulting to DICT_4X4_50")
            name = 'DICT_4X4_50'
        return aruco.getPredefinedDictionary(mapping[name])

    def camera_info_callback(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        D = np.array(msg.d, dtype=np.float64)
        self.camera_matrix = K
        self.dist_coeffs = D
        self.camera_frame_id = msg.header.frame_id  # ideally camera optical frame

    # --- Image callbacks ---
    def compressed_image_callback(self, msg: CompressedImage):
        if self.camera_matrix is None or self.dist_coeffs is None or self.camera_frame_id is None:
            return
        try:
            # Decode JPEG/PNG -> BGR
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"compressed_imgmsg_to_cv2 error: {e}")
            return
        self._process_and_publish(cv_img, msg.header)

    def image_callback(self, msg: Image):
        if self.camera_matrix is None or self.dist_coeffs is None or self.camera_frame_id is None:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"imgmsg_to_cv2 error: {e}")
            return
        self._process_and_publish(cv_img, msg.header)

    # --- Core pipeline ---
    def _process_and_publish(self, cv_img, header):
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None or len(ids) == 0:
            if self.publish_debug_image:
                self._publish_debug_image(cv_img)
            return

        try:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )
        except Exception as e:
            self.get_logger().error(f"Pose estimation failed: {e}")
            return

        
        stamp = header.stamp if self.use_image_header_stamp else self.get_clock().now().to_msg()

        for i, marker_id in enumerate(ids.flatten()):
            rvec = rvecs[i].reshape(3)
            R_o, _ = cv2.Rodrigues(rvec)
            tvec_o = tvecs[i].reshape(3)   # pose in optical frame
            

            # Convert translation and rotation to your camera frame
            tvec_c = self.R_co @ tvec_o
            R_c = self.R_co @ R_o
            qx, qy, qz, qw = self._rotm_to_quaternion(R_c)
            tx, ty, tz = tvec_c.tolist()

            child_id = f"{self.child_frame_prefix}{int(marker_id)}"
            if self.tf_prefix:
                child_id = f"{self.tf_prefix}/{child_id}"

            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.camera_frame_id   # your camera frame id
            tf_msg.child_frame_id = child_id

            tf_msg.transform.translation.x = float(tx)
            tf_msg.transform.translation.y = float(ty)
            tf_msg.transform.translation.z = float(tz)
            tf_msg.transform.rotation.x = float(qx)
            tf_msg.transform.rotation.y = float(qy)
            tf_msg.transform.rotation.z = float(qz)
            tf_msg.transform.rotation.w = float(qw)

            self.tf_broadcaster.sendTransform(tf_msg)


        if self.publish_debug_image:
            out = cv_img.copy()
            aruco.drawDetectedMarkers(out, corners, ids)
            for i in range(len(ids)):
                aruco.drawAxis(out, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.marker_length * 0.5)
            self._publish_debug_image(out)

    def _publish_debug_image(self, cv_img):
        if self.debug_pub is None:
            return
        try:
            ros_img = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            ros_img.header.stamp = self.get_clock().now().to_msg()
            ros_img.header.frame_id = self.camera_frame_id if self.camera_frame_id else ''
            self.debug_pub.publish(ros_img)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish debug image: {e}")

    @staticmethod
    def _rotm_to_quaternion(R: np.ndarray):
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0.0:
            S = math.sqrt(tr + 1.0) * 2.0
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        return (qx, qy, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
