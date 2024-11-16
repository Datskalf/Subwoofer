from adafruit_servokit import Servo

class Leg():
    servo_hip: Servo
    servo_upper: Servo
    servo_lower: Servo
    hip_center: float = None
    upper_center: float = None
    lower_center: float = None

    def __init__(self, servo_hip: Servo, servo_upper: Servo, servo_lower: Servo, flip_height: bool = False):
        self.servo_hip = servo_hip
        self.servo_upper = servo_upper
        self.servo_lower = servo_lower

        self.flip_height = flip_height



    def _get_servo(self, servo_index: int) -> Servo:
        """
        Fetches a servo based in the index given.
        If the index is out of bounds, will raise an IndexError.

        :param int servo_index: Index of the servo in range [0,2].
        :returns Servo: The servo given by the index passed.
        :raises IndexError: If servo index is out of bounds.
        """
        if servo_index == 0:
            return self.servo_hip
        elif servo_index == 1:
            return self.servo_upper
        elif servo_index == 2:
            return self.servo_lower
        else:
            raise IndexError
        
    def _set_center(self, servo_index: int, center: float) -> bool:
        """
        Sets the center value for a given server.
        Returns whether the setting was successful.
        """
        if center < 0 or center >= self._get_servo(servo_index).actuation_range:
            return False
        
        match servo_index:
            case 0:
                self.hip_center = center
                return True
            case 1:
                self.upper_center = center
                return True
            case 2:
                self.lower_center = center
                return True
            case _:
                raise IndexError
            
    def _set_all_centers(self, center_values: list[float]) -> None:
        self._set_center(0, center_values[0])
        self._set_center(1, center_values[1])
        self._set_center(2, center_values[2])

    def _get_center(self, servo_index: int) -> float:
        """
        Fetches the center angle for a given servo.
        If servo index is out of bounds, raises an IndexError.

        :param int servo_index: Index of the servo in range [0,2].
        :raises IndexError: If servo index is out of bounds
        """
        if servo_index == 0:
            return self.hip_center
        elif servo_index == 1:
            return self.upper_center
        elif servo_index == 2:
            return self.lower_center
        else:
            raise IndexError
        
    def set_servo(self, servo_index: int, angle: float, relative: bool = False) -> None:
        """
        Sets a specified servo to a given angle.
        If the servo index is out of bounds, will throw an IndexError.
        If angle is out of bounds, will bound it and go to the edge.

        :param int servo_index: Index of the servo in range [0,2].
        :param float angle: Angle to set the servo to.
        :param bool relative: If the angle should be an offset from center position.
        :raises IndexError: If servo index is out of bounds.
        """
        servo: Servo = self._get_servo(servo_index)

        if relative:
            angle = angle + self._get_center(servo_index)
        
        if angle < 0:
            servo.angle = 0
            return
        if angle >= servo.actuation_range:
            servo.angle = servo.actuation_range - 1
            return
        
        servo.angle = angle

    def set_all_abs(self, angle: float) -> None:
        """
        Runs the set_servo orutine on all the servos in this leg.

        :param float angle: Angle to set the servos to.
        """
        for i in range(3):
            self.set_servo(i, angle, False)

    def set_all_rel(self, angle: float) -> None:
        for i in range(3):
            self.set_servo(i, angle, True)
        
    def center_servo(self, servo_index: int) -> None:
        """
        Sets the given servo to its center position, if one exists.
        Otherwise, do nothing

        :param int servo_index: Index of the servo in range [0,2].
        """
        center = self._get_center(servo_index)
        if center is None:
            return
        self.set_servo(servo_index, center)

    def center_all(self, offset: float) -> None:
        self.set_servo(0, 0.0, True)
        if self.flip_height:
            offset *= -1
        self.set_servo(1, offset, True)
        self.set_servo(2, -2*offset, True)