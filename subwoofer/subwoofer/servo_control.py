# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
Lorem ipsum.

Dolor sit amet
"""

import rclpy
from rclpy.node import Node
import sys

from std_msgs.msg import Int32

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

        self.servo_update = self.create_subscription(Int32,
                                                 f"{self.get_name()}/set_angle",
                                                 self.servo_update,
                                                 10)

    def servo_update(self, msg: Int32) -> None:
        self.get_logger().info(f"Received value {msg.data}")
        self.servo.move_to(msg.angle)


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
