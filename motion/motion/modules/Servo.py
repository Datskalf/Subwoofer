from adafruit_pca9685 import PWMChannel

class Servo():
    """
    A custom implementation of the adafruit Servo class which allows for a wider range of motion.

    """

    def __init__(self, channel: PWMChannel, is_flipped: bool = False) -> None:
        """
        Instansiates a new Servo object tied to a specific I2C channel.

        :param adafruit_pca9685.PWMChannel channel: Channel for the physical servo.
        :param bool is_flipped: If the servo angle need to be raised or lowered when raising the robot.
        """
        
        self.channel = channel
        self.min_angle = 0
        self.max_angle = 180
        self.min_duty_cycle = 2000
        self.max_duty_cycle = 10000
        self.duty_cycle_per_degree: float = 44.4444
        self.duty_cycle_at_0 = 2000
        self.current_angle = None
        self.is_flipped = is_flipped
        self.default_angle = 90
        ...

    def set_angle(self, angle: int) -> None:
        """
        Moves the conencted servo to the specific angle.
        
        If the given angle is outside the actuation range of the servo,
        the angle is capped.

        :param int angle: Angle to send the servo to.
        """

        if self.is_flipped:
            angle = self.max_angle - angle
        
        if angle < self.min_angle:
            angle = self.min_angle
        if angle > self.max_angle:
            angle = self.max_angle

        self.current_angle = angle
        self.channel.duty_cycle = int(angle / self.max_angle * (self.max_duty_cycle - self.min_duty_cycle) + self.min_duty_cycle)
    
    def get_angle(self) -> int | None:
        """
        Fetches the current servo angle, if one exists.
        Otherwise, returns nothing

        :returns: Current servo angle
        :rtype: int | None
        """

        return self.current_angle
    
    def set_default_angle(self, angle: int) -> None:
        """
        Stores at what angle the default position is for standing.

        :param int angle: What angle should be default.
        """

        if angle < self.min_angle:
            self.default_angle = self.min_angle
            return
        if angle > self.max_angle:
            self.default_angle = self.max_angle
            return
        self.default_angle = angle

    def set_current_as_0(self):
        """
        TODO
        """
        ...

    def move_to(self, angle: int) -> None:
        """
        Moves the servo to the specified angle.

        :param int angle: What angle the servo should go to.
        """

        self.set_angle(angle)

    def move_by(self, angle: int):
        """
        Moves the servo by a given angle.

        :param int angle: How far the servo should rotate.
        """

        self.set_angle(self.current_angle + angle)