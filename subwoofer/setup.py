from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'subwoofer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*"))
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sondre Flakstad',
    maintainer_email='SondreFlakstad@outlook.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "servo = subwoofer.servo_control:main",
            "leg = subwoofer.leg_control:main",
            "FaceDetection = subwoofer.face_detection:main",
        ],
    },
)
