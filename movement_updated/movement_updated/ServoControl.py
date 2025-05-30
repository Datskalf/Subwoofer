import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from interfaces_updated.msg import ServoCommand
from .modules.Servo import Servo


class ServoControl(Node):
    last_angle: Float64 = 0.0
    target_angle: Float64 = 0.0
    degrees_per_second: int = 10
    servo: Servo|None = None

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
            self.servo.move_to(0)


        self.servo_sub = self.create_subscription(
            ServoCommand,
            f"/subwoofer/MoveLegs/{self.leg_name}",
            self.receive_servo,
            10
        )

        self.servo_pub = self.create_publisher(
            Float64,
            f"/subwoofer/Legs/{self.leg_name}/{self.servo_name}_controller",
            10
        )

        self.servo_timer = self.create_timer(
            100,
            self.move_servo
        )

    def set_move_command(self, angle: int, time: int = None) -> None:
        if time is None:
            time = 2
        if time <= 0:
            self.servo.move_to(angle)
            return
        d_angle = self.last_angle - angle
        self.degrees_per_second = d_angle / time
        self.target_angle = angle


    def move_servo(self) -> None:
        if self.servo_number == "-1":
            return
        
        # Move servo towards its goal
        if self.servo is not None:
            

            return
    
    def receive_move_speed(self, msg) -> None:
        ...

    def receive_servo(self, msg: ServoCommand) -> None:
        self.last_angle = msg.Angles[self.servo_number]



def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
