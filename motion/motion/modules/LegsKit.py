"""
TODO
"""

from .Leg import Leg

from adafruit_servokit import ServoKit
from adafruit_pca9685 import PCA9685
import board
from busio import I2C
from .Servo import Servo

class LegsKit():
    """
    TODO
    """

    front_left: Leg = None
    front_right: Leg = None
    back_left: Leg = None
    back_right: Leg = None

    def __init__(self, servo_addresses: list[list[int]], center_values: list[list[float]] | None = None) -> None:
        """
        TO BE DEPRECATED
        Initialises a LegsKit object.
        This initialiser will take a 2d array of I2C addresses, create each leg individually and store them is one LegsKit object.

        :param list[list[int]] servo_addresses: A 2D array of I2C addresses.
        :param list[list[float]] | None center_values: (Default None) Defines where the default values for the servos should be positioned. 
        """

        self.kit = ServoKit(channels=16)

        self.front_left = Leg(
            self.kit.servo[servo_addresses[0][0]],
            self.kit.servo[servo_addresses[0][1]],
            self.kit.servo[servo_addresses[0][2]]
        )
        self.front_right = Leg(
            self.kit.servo[servo_addresses[1][0]],
            self.kit.servo[servo_addresses[1][1]],
            self.kit.servo[servo_addresses[1][2]],
            True
        )
        self.back_left = Leg(
            self.kit.servo[servo_addresses[2][0]],
            self.kit.servo[servo_addresses[2][1]],
            self.kit.servo[servo_addresses[2][2]]
        )
        self.back_right = Leg(
            self.kit.servo[servo_addresses[3][0]],
            self.kit.servo[servo_addresses[3][1]],
            self.kit.servo[servo_addresses[3][2]],
            True
        )

        if center_values is not None:
            self.front_left._set_all_centers(center_values[0])
            self.front_right._set_all_centers(center_values[1])
            self.back_left._set_all_centers(center_values[2])
            self.back_right._set_all_centers(center_values[3])

    def __init__(self) -> None:
        """
        Creates a new LegsKit object using the custom .modules.Servo implementation.
        """

        self.pca = PCA9685(I2C(board.SCL, board.SDA))
        self.pca.frequency = 60

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

        

    def stand(self, height: float = 0.0) -> None:
        """
        Sets all the legs to a standing position.
        The height of said standing can be adjusted with the height parameter.

        :param float height: What the height should be adjusted by.
        """

        for i, leg in enumerate([self.front_left, self.front_right, self.back_left, self.back_right]):
            print(f"Leg {i}:")
            leg.set_stand(height)