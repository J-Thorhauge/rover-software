
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoReceiver(Node):
    def __init__(self):
        super().__init__('video_receiver')
        self.publisher_ = self.create_publisher(Image, 'gstreamer_video', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(
            'udpsrc port=5000 caps="application/x-rtp" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink',
            cv2.CAP_GSTREAMER
        )
        self.timer = self.create_timer(0.033, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VideoReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
