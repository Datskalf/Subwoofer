"""
Lorem ipsum.

Dolor sit amet
"""

import rclpy
from rclpy.node import Node
import sys

from std_msgs.msg import Int32, Float32

from subwoofer.servo_pkg.Servo import Servo


class ServoControl(Node):
    def __init__(self):
        super().__init__("servo")

        self.declare_parameter("simulated", False)
        self.declare_parameter("pwm_channel", -1)
        self.declare_parameter("min_duty_cycle", 2000)
        self.declare_parameter("max_duty_cycle", 10000)
        self.declare_parameter("duty_cycle_per_degree", 44.4444)
        self.declare_parameter("min_angle", -90)
        self.declare_parameter("max_angle", 90)
        self.declare_parameter("flipped", False)

        self.is_simulated = self.get_parameter("simulated")
        self.pwm_channel = self.get_parameter("pwm_channel")
        self.min_duty_cycle = self.get_parameter("min_duty_cycle")
        self.max_duty_cycle = self.get_parameter("max_duty_cycle")
        self.duty_cycle_per_degree = self.get_parameter("duty_cycle_per_degree")
        self.min_angle = self.get_parameter("min_angle")
        self.max_angle = self.get_parameter("max_angle")
        self.is_flipped = self.get_parameter("flipped")

        self.servo = Servo(self.pwm_channel, self.is_flipped, self.is_simulated)

        # Make into an action
        #self.servo_update = self.create_subscription(Int32,
        #                                         f"{self.get_name()}/set_angle",
        #                                         self.servo_update,
        #                                         10)
        
        self.pub_angle = self.create_publisher(Float32,
                                                     f"{self.get_name()}/current_angle",
                                                     10)
        
        self.pub_angle_timer = self.create_timer(0.5, self.angle_pub)

    def angle_pub(self) -> None:
        msg = Float32()
        msg.data = self.servo.get_angle()
        self.pub_angle.publish(msg)

    def servo_update(self, msg: Float32) -> None:
        self.get_logger().info(f"Received value {msg.data}")
        self.servo.move_to(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
