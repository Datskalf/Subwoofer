from .Leg import Leg

from adafruit_servokit import ServoKit, Servo

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
            self.kit.servo[servo_addresses[1][2]]
        )
        self.back_left = Leg(
            self.kit.servo[servo_addresses[2][0]],
            self.kit.servo[servo_addresses[2][1]],
            self.kit.servo[servo_addresses[2][2]]
        )
        self.back_right = Leg(
            self.kit.servo[servo_addresses[3][0]],
            self.kit.servo[servo_addresses[3][1]],
            self.kit.servo[servo_addresses[3][2]]
        )

        if center_values is not None:
            self.front_left._set_all_centers(center_values[0])
            self.front_right._set_all_centers(center_values[1])
            self.back_left._set_all_centers(center_values[2])
            self.back_right._set_all_centers(center_values[3])