"""Package file containing the custom Servo class."""

from adafruit_pca9685 import PWMChannel
from adafruit_pca9685 import PCA9685
import board
from busio import I2C

class Servo():
    """A custom implementation of the adafruit Servo class which allows for a wider range of motion."""

    def __init__(self, channel: int, is_flipped: bool = False, is_simulated: bool = False) -> None:
        """
        Instansiates a new Servo object tied to a specific I2C channel.

        :param adafruit_pca9685.PWMChannel channel: Channel for the physical servo.
        :param bool is_flipped: If the servo angle need to be raised or lowered when raising the robot.
        """
        self.is_simulated = is_simulated

        if not is_simulated:
            self.pca = PCA9685(I2C(board.SCL, board.SDA))
            self.pca.frequency = 60
            self.channel: PWMChannel = PWMChannel(self.pca, channel)

        self.min_duty_cycle: int = 2000
        self.max_duty_cycle: int = 10000

        self.min_angle: int = -90
        self.max_angle: int = 90

        self.current_angle: float = 0.0
    
    def move_to(self, angle: float) -> None:
        """Move the servo to the specified angle."""
        angle = self._cap_angle(angle)
        dc = self._angle_to_duty_cycle(angle)
        if not self.is_simulated:
            self.channel.duty_cycle = dc
        self.current_angle = angle

    def move_by(self, angle: float) -> None:
        """Move the servo by the given angle relative to its current position."""
        angle = self._cap_angle(self.current_angle + angle)
        self.move_to(angle)

    def get_angle(self) -> float:
        return float(self.current_angle)
        ...

    def _cap_angle(self, angle: int) -> int:
        """Caps the angle to the servo threshold (default -90, 90 inclusive)."""
        if angle < self.min_angle:
            return self.min_angle
        if angle > self.max_angle:
            return self.max_angle
        return angle

    def _angle_to_duty_cycle(self, angle: int) -> float|int:
        """Convert an angle to its respective duty cycle."""
        angle_offset_from_min = angle - self.min_angle
        
        duty_cycle_per_degree = (self.max_duty_cycle-self.min_duty_cycle)/(self.max_angle-self.min_angle)
        duty_cycle_offset = angle_offset_from_min * duty_cycle_per_degree
        
        return duty_cycle_offset + self.min_duty_cycle
    
    def _duty_cycle_to_angle(self, duty_cycle: int) -> float|int:
        """Convert a duty cycle to its respective angle."""
        dc_offset = duty_cycle - self.min_duty_cycle
        degree_per_dc = (self.max_angle-self.min_angle)/(self.max_duty_cycle-self.min_duty_cycle)
        return dc_offset * degree_per_dc + self.min_angle
        

def main():
    """Can define test cases here."""
    servo = Servo(1)
    for i in range(-100, 120, 20):
        print(f"Angle {i}: output {servo._cap_angle(i)}")
    print(servo._angle_to_duty_cycle(45))

if __name__ == "__main__":
    main()