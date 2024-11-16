import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, Image

import cv2
from cv_bridge import CvBridge


class CameraFeed(Node):
    def __init__(self):
        super().__init__("camera_feed")

        self.bridge = CvBridge()
        self.camera = cv2.VideoCapture("/dev/video0")


        self.camera_feed_pub = self.create_publisher(
            Image,
            "/subwoofer/sensors/camera_0/image",
            10
        )

        self.camera_feed_timer = self.create_timer(
            0.5,
            self.camera_feed_callback
        )

    def camera_feed_callback(self):
        result, image = self.camera.read()

        if not result:
            return
        
        ros_img = self.bridge.cv2_to_compressed_imgmsg(image)
        #ros_img = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
        self.camera_feed_pub.publish(ros_img)
        self.get_logger().info(f"Publishing image...")
        ...


def main(args=None):
    rclpy.init(args=args)
    node = CameraFeed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()