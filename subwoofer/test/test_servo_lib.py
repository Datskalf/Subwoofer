def test_angle_to_dc():
    from subwoofer.servo_pkg.Servo import Servo

    servo = Servo(-1, False, True)
    
    ang_range = servo.max_angle - servo.min_angle
    dc_range = servo.max_duty_cycle - servo.min_duty_cycle

    for i in range(11):
        assert servo._angle_to_duty_cycle(servo.min_angle + i * ang_range/10) == servo.min_duty_cycle + i * dc_range/10

def test_dc_to_angle():
    from subwoofer.servo_pkg.Servo import Servo

    servo = Servo(-1, False, True)

    ang_range = servo.max_angle - servo.min_angle
    dc_range = servo.max_duty_cycle - servo.min_duty_cycle

    for i in range(11):
        assert servo._duty_cycle_to_angle(servo.min_duty_cycle + i * dc_range/10) == servo.min_angle + i * ang_range/10