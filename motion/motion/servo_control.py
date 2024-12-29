"""
Contains the main control code for servo controls.
"""

import rclpy
from rclpy.node import Node

from interfaces.msg import ServoAngle
from interfaces.msg import ServoAngles
from std_msgs.msg import Float32

from .modules.LegsKit import LegsKit

class ServoControl(Node):
    """
    This node is mainly responsible for controlling how the servos move on the Subwoofer robot dog.
    """
    
    legs: LegsKit | None = None

    def __init__(self):
        """
        Initialises the ServoControl node with the node name "servo_control"
        The initialiser will hold a LegsKit object, allowing it to manipulate the robot's servos systematically.
        """
        
        super().__init__("servo_control")

        self.valid_servo_indices = [0,1,2,3,4,5,6,7,12,13,14,15]

        self.legs = LegsKit()


        # Subscriptions
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
        Sets a specific servo to a given angle.

        :param interfaces.msg.ServoAngle msg: Message with the servo angle and id
        """
        
        if self.legs is None:
            return
        
        # Log servo update
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
        Publishes all servo angles to its own topic.
        If unable to get angles, publishes all as negative
        """
        
        msg = ServoAngles()
            
        try:
            msg.angles = [
                float(self.legs.front_left.servo_hip.get_angle()),
                float(self.legs.front_left.servo_upper.get_angle()),
                float(self.legs.front_left.servo_lower.get_angle()),

                float(self.legs.front_right.servo_hip.get_angle()),
                float(self.legs.front_right.servo_upper.get_angle()),
                float(self.legs.front_right.servo_lower.get_angle()),

                float(self.legs.back_left.servo_hip.get_angle()),
                float(self.legs.back_left.servo_upper.get_angle()),
                float(self.legs.back_left.servo_lower.get_angle()),

                float(self.legs.back_right.servo_hip.get_angle()),
                float(self.legs.back_right.servo_upper.get_angle()),
                float(self.legs.back_right.servo_lower.get_angle()),
            ]
        except:
            msg.angles = [-1.0] * 12
        self.servo_angle_pub.publish(msg)

        

    def servo_update_all(self, msg: ServoAngles):
        """
        Takes in an array of servo angles and sets the servos to said angles in the order of:
        - front left hip
        - front left upper
        - front left lower
        - front right hip
        - front right upper
        - front right lower
        - back left hip
        - back left upper
        - back left lower
        - back right hip
        - back right upper
        - back right lower

        If less than 12 elements is provided, the message will be discarded.
        If any angles are out of range, they will be capped by the individual servos' actuation range.

        :param interfaces.msg.ServoAngles msg: Message data containing all angles.
        """
        
        if self.legs is None:
            return
        if len(msg.angles) < 12:
            return
        
        self.legs.set_all_servos(msg.angles[:12])
        
        

    def set_standing(self, msg: Float32):
        """
        TODO Not working
        Sets all servo angles to their default angle for standing.
        """
        
        self.get_logger().info(f"Moving servos to standing")
        self.legs.stand(msg.data)



def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()