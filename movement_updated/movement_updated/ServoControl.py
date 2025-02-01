import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from interfaces_updated.msg import ServoCommand
from .modules.Servo import Servo


class ServoControl(Node):
    last_angle: Float64 = 0

    def __init__(self):
        super().__init__("servo_control")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("servo_name", "Servo_x"),
                ("servo_number", -1),
                ("leg_name", "Leg_x"),
                ("leg_number", -1),
                ("pwm_channel", -1)
            ]
        )

        self.servo_name = self.get_parameter("servo_name").value
        self.servo_number = self.get_parameter("servo_number").value
        self.leg_name = self.get_parameter("leg_name").value
        self.leg_number = self.get_parameter("leg_number").value
        self.pwm_channel = self.get_parameter("pwm_channel").value


        # Create servo connection
        if self.pwm_channel != -1:
            self.servo = Servo(self.pwm_channel)


        self.servo_sub = self.create_subscription(
            ServoCommand,
            f"/subwoofer/MoveLegs/{self.leg_name}",
            self.receive_servo,
            10
        )

        self.servo_pub = self.create_publisher(
            Float64,
            f"/subwoofer/Legs/{self.leg_name}/{self.servo_name}_position_controller/command",
            10
        )

        self.servo_timer = self.create_timer(
            0.1,
            self.move_servo
        )


    def move_servo(self) -> None:
        if self.servo_number == "-1":
            return
        
        self.get_logger().info("Updating servo angle")
        return

    def receive_servo(self, msg: ServoCommand) -> None:
        self.last_angle = msg.Angles[self.servo_number]



def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
