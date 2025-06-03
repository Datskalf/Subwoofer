def test_angle_to_dc():
    from subwoofer.servo_pkg.Servo import Servo

    servo = Servo(-1, False, True)

    assert servo._angle_to_duty_cycle(servo.min_angle) == servo.min_duty_cycle
    assert servo._angle_to_duty_cycle(servo.max_angle) == servo.max_duty_cycle
    assert servo._angle_to_duty_cycle((servo.min_angle + servo.max_angle) / 2) == (servo.min_duty_cycle + servo.max_duty_cycle) / 2
    