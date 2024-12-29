"""
Package file containing the Leg class.
"""

from .Servo import Servo

class Leg():
    """
    The Leg object allows for a grouping of three servos mounted inside a single leg on the robot.
    Using this class, all three servos can be controlled from the same point.
    """
    
    servo_hip: Servo
    servo_upper: Servo
    servo_lower: Servo

    def __init__(self, servo_hip: Servo, servo_upper: Servo, servo_lower: Servo):
        """
        Initialises a new Leg object using the three provided servo objects.

        :param .modules.Servo servo_hip: The servo controlling the roll of the leg
        :param .modules.Servo servo_upper: The upper servo controlling pitch
        :param .modules.Servo servo_lower: The lower servo controlling pitch
        """
        
        self.servo_hip = servo_hip
        self.servo_upper = servo_upper
        self.servo_lower = servo_lower

        self.servo_hip.default_angle = 90
        self.servo_upper.default_angle = 135
        self.servo_lower.default_angle = 90


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

        # If servo doesn't exist, don't do anything
        try:
            servo: Servo = self._get_servo(servo_index)
        except IndexError:
            return
        
        # Set servo
        if relative:
            servo.move_by(angle)
        else:
            servo.move_to(angle)

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

        servo = self._get_servo(servo_index)
        servo.move_to(servo.default_angle)


    def set_stand(self, height: float) -> None:
        """
        Sets the leg in a standing position.

        :param float height: Height offset for the leg.
        """
        
        self.servo_hip.move_to(self.servo_hip.default_angle)
        self.servo_upper.move_to(self.servo_upper.default_angle + height)
        self.servo_lower.move_to(self.servo_lower.default_angle - height*2)