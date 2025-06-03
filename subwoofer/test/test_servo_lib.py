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

def test_angle_capping():
    from subwoofer.servo_pkg.Servo import Servo

    servo = Servo(-1, False, True)

    for ang in range(servo.min_angle - 10, servo.max_angle + 10, 5):
        cap_ang = servo._cap_angle(ang)
        assert cap_ang >= servo.min_angle and cap_ang <= servo.max_angle
        if ang < servo.min_angle:
            assert cap_ang == servo.min_angle
        elif ang > servo.max_angle:
            assert cap_ang == servo.max_angle
        else:
            assert cap_ang == ang
