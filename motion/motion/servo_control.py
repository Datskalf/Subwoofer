import rclpy
from rclpy.node import Node
from interfaces.msg import ServoAngle

from adafruit_servokit import ServoKit

class ServoControl(Node):
    kit: ServoKit | None = None
    def __init__(self):
        super().__init__("servo_control")
        self.get_logger().info(f"Servo control online!")

        try:
            self.kit = ServoKit(channels=16)
        except ValueError as ex:
            self.get_logger().error(f"{ex}")
            ...


        self.subscriber = self.create_subscription(
            ServoAngle,
            "/subwoofer/servos/servo_update",
            self.servo_update,
            10
        )

        self.get_logger().info(f"Servo controller online!")



    def servo_update(self, msg: ServoAngle):
        if self.kit is None:
            return
        
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt as ex:
        pass
    node.destroy_node()
    rclpy.shutdown()