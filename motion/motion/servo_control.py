import rclpy
from rclpy.node import Node

from interfaces.msg import ServoAngle
from std_msgs.msg import Float32

from adafruit_servokit import ServoKit, Servo

from .modules.Leg import Leg
from .modules.LegsKit import LegsKit

class ServoControl(Node):
    kit: ServoKit | None = None
    legs: dict[str: Leg] | None = None

    def __init__(self):
        super().__init__("servo_control")

        self.valid_servo_indices = [0,1,2,3,4,5,6,7,12,13,14,15]
        values = [70,132,130,78, 105,140,110,90, 179,100,110,48]

        try:
            self.kit = ServoKit(channels=16)
        except ValueError as ex:
            self.get_logger().error(f"{ex}")
            return
            ...

        self.legs = LegsKit([
                [ 5,  2,  0],
                [ 4,  3,  1],
                [ 7, 13, 15],
                [ 6, 12, 14]
            ], [
                [140, 130,  70],
                [105,  78, 132],
                [ 90, 100,  48],
                [110, 179, 110]
            ])




        self.servo_individual_control = self.create_subscription(
            ServoAngle,
            "/subwoofer/servos/servo_update",
            self.servo_update,
            10
        )

        self.servo_joint_control = self.create_subscription(
            Float32,
            "/subwoofer/servos/servo_joint_update",
            self.servo_update_all,
            10
        )

        self.set_standing = self.create_subscription(
            Float32,
            "/subwoofer/servos/standing_height",
            self.set_standing,
            10
        )

        self.get_logger().info(f"Servo controller online!")



    def servo_update(self, msg: ServoAngle):
        if self.kit is None:
            return
        
        self.get_logger().info(f"Servo update: {msg.index} {msg.angle}")
        if msg.index not in self.valid_servo_indices:
            return
        servo = self.kit.servo[msg.index]
        if msg.angle not in range(servo.actuation_range):
            return
        
        servo.angle = msg.angle

    def servo_update_all(self, msg: Float32):
        if self.kit is None:
            return
        
        self.get_logger().info(f"Setting all servos to {msg.data}")

        for index in self.valid_servo_indices:
            servo = self.kit.servo[index]
            if msg.data not in range(servo.actuation_range):
                continue
            servo.angle = msg.data

    def set_standing(self, msg: Float32):
        return
        values = [70,132,130,78,105,140,110,90,179,100,110,48]
        self.get_logger().info(f"Moving servos to standing with offset {msg.data}")
        for i, index in enumerate(self.valid_servo_indices):
            angle = values[i]
            try:
                if i in [5,4,7,6]:
                    self.kit.servo[index].angle = msg.data
                elif i in [2,1,13,12]:
                    self.kit.servo[index].angle = msg.data + angle
                else:
                    self.kit.servo[index].angle = msg.data - angle
            except ValueError as ex:
                pass



def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt as ex:
        pass
    node.destroy_node()
    rclpy.shutdown()