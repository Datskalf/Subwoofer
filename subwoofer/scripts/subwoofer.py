import rclpy
from rclpy.node import Node

from subwoofer.servo_pkg.Servo import Servo

from adafruit_pca9685 import PWMChannel
from adafruit_pca9685 import PCA9685
import board
from busio import I2C


class Subwoofer(Node):
    def __init__(self):
        super().__init__("subwoofer")

        self.declare_parameter("simulated", False)
        self.is_simulated = self.get_parameter("simulated")
        self.declare_parameter("pwm_channel", -1)
        self.pwm_channel = self.get_parameter("pwm_channel")
        self.declare_parameter("min_duty_cycle", 2000)
        self.min_duty_cycle = self.get_parameter("min_duty_cycle")
        self.declare_parameter("max_duty_cycle", 10000)
        self.max_duty_cycle = self.get_parameter("max_duty_cycle")
        self.declare_parameter("duty_cycle_per_degree", 44.4444)
        self.duty_cycle_per_degree = self.get_parameter("duty_cycle_per_degree")
        self.declare_parameter("min_angle", -90)
        self.min_angle = self.get_parameter("min_angle")
        self.declare_parameter("max_angle", 90)
        self.max_angle = self.get_parameter("max_angle")
        self.declare_parameter("flipped", False)
        self.is_flipped = self.get_parameter("flipped")

        self.servo = Servo(self.pwm_channel, self.is_flipped)



def main(args=None):
    rclpy.init(args=args)
    node = Subwoofer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
