import rclpy
from rclpy.node import Node

class WebGUI(Node):
    def __init__(self):
        super().__init__("web_gui")

        


def main(args=None):
    rclpy.init(args=args)
    node = WebGUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()