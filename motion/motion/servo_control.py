import rclpy
from rclpy.node import Node

class ServoControl(Node):
    def __init__(self):
        super().__init__("servo_control")
        self.get_logger().info(f"Servo control online!")



def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()