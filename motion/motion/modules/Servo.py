from adafruit_pca9685 import PWMChannel

class Servo():
    def __init__(self, channel: PWMChannel, is_flipped: bool = False):
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

    def set_angle(self, angle: int):
        if self.is_flipped:
            angle = self.max_angle - angle
        
        if angle < self.min_angle:
            angle = self.min_angle
        if angle > self.max_angle:
            angle = self.max_angle

        self.current_angle = angle
        self.channel.duty_cycle = int(angle / self.max_angle * (self.max_duty_cycle - self.min_duty_cycle) + self.min_duty_cycle)
    
    def get_angle(self) -> int | None:
        return self.current_angle
    
    def set_default_angle(self, angle: int):
        if angle < self.min_angle:
            self.default_angle = self.min_angle
            return
        if angle > self.max_angle:
            self.default_angle = self.max_angle
            return
        self.default_angle = angle

    def set_current_as_0(self):
        ...

    def move_to(self, angle: int):
        self.set_angle(angle)

    def move_by(self, angle: int):
        self.set_angle(self.current_angle + angle)