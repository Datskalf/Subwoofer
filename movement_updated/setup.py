from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'movement_updated'

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
    maintainer='sondre',
    maintainer_email='Datskalf@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ServoControl = movement_updated.ServoControl:main',
            'LegControl = movement_updated.Leg:main'
        ],
    },
)
