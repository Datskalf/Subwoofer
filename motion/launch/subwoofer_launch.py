from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motion',
            namespace='subwoofer_motion',
            executable='servo_control',
            name='Servos'
        )
    ])