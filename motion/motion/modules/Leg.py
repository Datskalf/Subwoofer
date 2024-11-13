from adafruit_servokit import Servo

class Leg():
    def __init__(self, servo_hip: Servo, servo_upper: Servo, servo_lower: Servo):
        self.servo_hip = servo_hip
        self.servo_upper = servo_upper
        self.servo_lower = servo_lower
        
        