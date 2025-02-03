"""
Package file containing the custom Servo class.
"""

from adafruit_pca9685 import PWMChannel
from adafruit_pca9685 import PCA9685
import board
from busio import I2C

class Servo():
    """
    A custom implementation of the adafruit Servo class which allows for a wider range of motion.
    """

    def __init__(self, channel: int, is_flipped: bool = False) -> None:
        """
        Instansiates a new Servo object tied to a specific I2C channel.

        :param adafruit_pca9685.PWMChannel channel: Channel for the physical servo.
        :param bool is_flipped: If the servo angle need to be raised or lowered when raising the robot.
        """

        self.pca = PCA9685(I2C(board.SCL, board.SDA))
        self.pca.frequency = 60

        self.channel: PWMChannel = PWMChannel(self.pca, channel)
        self.min_duty_cycle: int = 2000
        self.max_duty_cycle: int = 10000
        self.duty_cycle_per_degree: float = 44.4444

        self.min_angle: int = -90
        self.max_angle: int = 90

        self.current_angle: int = None


    
    def move_to(self, angle: int) -> None:
        """
        Moves the servo to the specified angle.
        """
        angle = self._cap_angle(angle)
        dc = self._angle_to_duty_cycle(angle)
        self.channel.duty_cycle = dc
        self.current_angle = angle

    def move_by(self, angle: int) -> None:
        """
        Moves the servo by the given angle relative to its current position.
        """
        angle = self._cap_angle(self.current_angle + angle)
        self.move_to(angle)



    def _cap_angle(self, angle: int) -> int:
        """
        Caps the angle to the servo threshold (default -90, 90 inclusive)
        """
        if angle < self.min_angle:
            return self.min_angle
        if angle > self.max_angle:
            return self.max_angle
        return angle

    def _angle_to_duty_cycle(self, angle: int) -> float|int:
        """
        Converts an angle to its respective duty cycle.
        TODO: uses a hardcoded dutycycle-to-angle value, could calculate this live instead.
        """
        angle = self._cap_angle(angle)
        angle_offset_from_min = angle - self.min_angle
        duty_cycle_offset = angle_offset_from_min * self.duty_cycle_per_degree
        return duty_cycle_offset + self.min_duty_cycle
        

def main():
    """
    Can define test cases here.
    """
    servo = Servo(1)
    for i in range(-100, 120, 20):
        print(f"Angle {i}: output {servo._cap_angle(i)}")
    print(servo._angle_to_duty_cycle(45))

if __name__ == "__main__":
    main()