import rclpy
from rclpy.node import Node

import board
from adafruit_mpu6050 import MPU6050

from interfaces.msg import MPU

class MPUMeasure(Node):
    def __init__(self):
        super().__init__("mpu_measure")

        self.i2c = board.I2C()
        self.mpu = MPU6050(self.i2c)

        self.timer = self.create_timer(
            0.1,
            self.mpu_publish
        )

        self.mpu_publisher = self.create_publisher(
            MPU,
            "/subwoofer/sensors/mpu_0",
            10
        )


    def mpu_publish(self):
        msg = MPU()
        msg.acceleration.x, msg.acceleration.y, msg.acceleration.z = self.mpu.acceleration
        msg.gyro.x, msg.gyro.y, msg.gyro.z = self.mpu.gyro
        msg.temperature = self.mpu.temperature
        self.mpu_publisher.publish(msg)
        ...

    


def main(args=None):
    rclpy.init(args=args)
    node = MPUMeasure()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()