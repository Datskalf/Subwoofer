from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*"))
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sondre Flakstad',
    maintainer_email='som38@aber.ac.uk',
    description='TODO: Package description',
    license='Apache license 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Servos = motion.servo_control:main',
        ],
    },
)
