import rclpy
from rclpy.node import Node



class CameraFeed(Node):
    def __init__(self):
        super().__init__("camera_feed")


def main(args=None):
    rclpy.init(args=args)
    node = CameraFeed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()