"""
Package file containing the LegsKit class.
"""

from .Leg import Leg

from adafruit_servokit import ServoKit
from adafruit_pca9685 import PCA9685
import board
from busio import I2C
from .Servo import Servo

class LegsKit():
    """
    The LegsKit object initialises with predefined I2C addresses for every servo,
    and bundles them together into Leg objects for easy interfacing.
    """

    front_left: Leg = None
    front_right: Leg = None
    back_left: Leg = None
    back_right: Leg = None

    def __init__(self) -> None:
        """
        Creates a new LegsKit object using the custom .modules.Servo implementation.
        """

        self.pca = PCA9685(I2C(board.SCL, board.SDA))
        self.pca.frequency = 60

        # Create each leg with their default I2C channels
        self.front_left = Leg(
            Servo(self.pca.channels[5]),
            Servo(self.pca.channels[2]),
            Servo(self.pca.channels[0]))
        self.front_right = Leg(
            Servo(self.pca.channels[4]),
            Servo(self.pca.channels[3], True),
            Servo(self.pca.channels[1], True))
        self.back_left = Leg(
            Servo(self.pca.channels[7]),
            Servo(self.pca.channels[13]),
            Servo(self.pca.channels[15]))
        self.back_right = Leg(
            Servo(self.pca.channels[6]),
            Servo(self.pca.channels[12], True),
            Servo(self.pca.channels[14], True))

        
    def get_legs(self) -> list[Leg]:
        return (self.front_left, self.front_right, self.back_left, self.back_right)


    def stand(self, height: float = 0.0) -> None:
        """
        Sets all the legs to a standing position.
        The height of said standing can be adjusted with the height parameter.

        :param float height: What the height should be adjusted by.
        """

        for i, leg in enumerate(self.get_legs()):
            print(f"Leg {i}:")
            leg.set_stand(height)


    def set_all_servos(self, angles: list[float]) -> None:
        """
        Sets each servo to an angle defined in the angles parameter.
        If an angle is set to -1, it will maintain its current angle.

        :param list[float] angles: List of angles to set each servo to.
        """
        
        for leg_index, leg in enumerate(self.get_legs()):
            for servo_index in range(3):
                angle = angles[leg_index*3 + servo_index]
                if angle == -1.0:
                    continue
                leg.set_servo(servo_index, angle)