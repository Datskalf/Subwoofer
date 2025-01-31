import rclpy
from rclpy.node import Node

from interfaces_updated.msg import ServoCommand, LegCommand


class Leg(Node):
    def __init__(self) -> None:
        super().__init__("servo_control")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("name", "Leg_x"),
                ("number", -1)
            ]
        )

        self.name = self.get_parameter(name="name").value
        self.number = self.get_parameter(name="number").value


        self.leg_publisher = self.create_publisher(
            ServoCommand,
            f"/subwoofer/Legs/{self.name}",
            10
        )

        self.leg_subscriber = self.create_subscription(
            LegCommand,
            f"/subwoofer/MoveLegs/{self.name}",
            self.move_leg,
            10
        )

        self.get_logger().info(f"Leg node {self.name} online")

    def move_leg(self, msg: LegCommand) -> None:
        # TODO
        ...



def main(args=None):
    rclpy.init(args=args)
    node = Leg()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()