"""
TODO
"""

import time
import rclpy
from rclpy.node import Node

from interfaces.msg import ServoAngle
from interfaces.msg import ServoAngles
from std_msgs.msg import Float32

from .modules.Leg import Leg
from .modules.LegsKit import LegsKit

class ServoControl(Node):
    """
    TODO
    """
    
    legs: LegsKit | None = None
    test: bool = False

    def __init__(self):
        """
        TODO
        """
        
        super().__init__("servo_control")

        self.valid_servo_indices = [0,1,2,3,4,5,6,7,12,13,14,15]

        self.legs = LegsKit()


        self.servo_individual_control = self.create_subscription(
            ServoAngle,
            "/subwoofer/servos/servo_update",
            self.servo_update,
            10
        )

        self.servo_joint_control = self.create_subscription(
            ServoAngles,
            "/subwoofer/servos/servo_joint_update",
            self.servo_update_all,
            10
        )

        self.set_standing_sub = self.create_subscription(
            Float32,
            "/subwoofer/servos/standing_height",
            self.set_standing,
            10
        )

        self.servo_angle_pub = self.create_publisher(
            ServoAngles,
            "/subwoofer/servos/current_angle",
            10
        )

        self.get_logger().info(f"Servo controller online!")



    def servo_update(self, msg: ServoAngle):
        """
        TODO
        """
        
        if self.legs is None:
            return
        
        self.get_logger().info(f"Servo update: {msg.leg}-{msg.servo} {msg.angle}")
        leg = None
        match msg.leg:
            case 0:
                leg = self.legs.front_left
            case 1:
                leg = self.legs.front_right
            case 2:
                leg = self.legs.back_left
            case 3:
                leg = self.legs.back_right
            case _:
                return
        
        leg.set_servo(msg.servo, msg.angle)

    def output_servo_values(self):
        """
        TODO
        """
        
        msg = ServoAngles()
            
        try:
            msg.angles = [
                float(self.legs.front_left.servo_hip.angle),
                float(self.legs.front_left.servo_upper.angle),
                float(self.legs.front_left.servo_lower.angle),

                float(self.legs.front_right.servo_hip.angle),
                float(self.legs.front_right.servo_upper.angle),
                float(self.legs.front_right.servo_lower.angle),

                float(self.legs.back_left.servo_hip.angle),
                float(self.legs.back_left.servo_upper.angle),
                float(self.legs.back_left.servo_lower.angle),

                float(self.legs.back_right.servo_hip.angle),
                float(self.legs.back_right.servo_upper.angle),
                float(self.legs.back_right.servo_lower.angle),
            ]
        except:
            msg.angles = [-1.0] * 12
        self.servo_angle_pub.publish(msg)

        

    def servo_update_all(self, msg: ServoAngles):
        """
        DEPRECATED: Change to use ServoAngles msg interface
        TODO
        """
        
        if self.legs is None:
            return
        if len(msg.angles) < 12:
            return
        
        self.legs.set_all_servos(msg.angles[:12])
        
        

    def set_standing(self, msg: Float32):
        """
        TODO
        """
        
        self.get_logger().info(f"Moving servos to standing")
        self.legs.stand(msg.data)



def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt as ex:
        pass
    node.destroy_node()
    rclpy.shutdown()