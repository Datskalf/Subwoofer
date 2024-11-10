import rclpy
from rclpy.node import Node
from interfaces.msg import ServoAngle

from adafruit_servokit import ServoKit

class ServoControl(Node):
    def __init__(self):
        super().__init__("servo_control")
        self.get_logger().info(f"Servo control online!")

        self.kit = ServoKit(channels=16)


        self.subscriber = self.create_subscription(
            ServoAngle,
            "servo_angle",
            self.servo_update,
            10
        )

    def servo_update(self, msg: ServoAngle):
        self.get_logger().info(f"Servo update: {msg.index} {msg.angle}")
        if msg.index not in [0,1,2,3,4,5,6,7,12,13,14,15]:
            return
        servo = self.kit.servo[msg.index]
        if msg.angle not in range(servo.actuation_range):
            return
        
        servo.angle = msg.angle



def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()