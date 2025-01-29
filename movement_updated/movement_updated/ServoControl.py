import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from interfaces_updated.msg import ServoCommand


class ServoControl(Node):
    last_angle: Float64 = 0

    def __init__(self, args):
        super().__init__("servo_control")

        self.leg_name = "NULL"
        self.servo_number = -1
        self.servo_name = "NULL"

        self.servo_sub = self.create_subscription(
            ServoCommand,
            f"/subwoofer/MoveLegs/dummy_leg",
            self.receive_servo,
            10
        )

        self.servo_pub = self.create_publisher(
            Float64,
            f"/subwoofer/servo_controller",
            10
        )

        self.servo_timer = self.create_timer(
            0.1,
            self.move_servo
        )


    def move_servo(self) -> None:
        if self.servo_number == -1:
            return
        
        msg_out = Float64()
        msg_out.data = self.last_angle
        self.servo_pub.publish(msg_out)

    def receive_servo(self, msg: ServoCommand) -> None:
        self.last_angle = msg.Angles[self.servo_number]



def main(args=None):
    rclpy.init(args=args)

    print(args)
    node = ServoControl(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
