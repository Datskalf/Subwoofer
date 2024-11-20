from .Leg import Leg

from adafruit_servokit import ServoKit
from Servo import Servo

class LegsKit():
    front_left: Leg = None
    front_right: Leg = None
    back_left: Leg = None
    back_right: Leg = None

    def __init__(self, servo_addresses: list[list[int]], center_values: list[list[float]] | None = None):
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

    def __init__(self):
        self.front_left = Leg(Servo(5), Servo(2), Servo(0))
        self.front_right = Leg(Servo(4), Servo(3, True), Servo(1, True))
        self.back_left = Leg(Servo(7), Servo(13), Servo(15))
        self.back_right = Leg(Servo(6), Servo(12, True), Servo(14, True))

    def stand(self, height = 0.0):
        for leg in [self.front_left, self.front_right, self.back_left, self.back_right]:
            leg.set_stand(height)