# Subwoofer

Subwoofer is a bipedal robot modelled after robots like Boston Dynamics' Spot and the Unitree go.
The version made has been given the name `spotmicro`, and the original designs can be found [here](https://gitlab.com/public-open-source/spotmicroai).

Software is currently ran on a raspberry pi running ROS2 Humble. Login for this pi is (username: `ubuntu`, password: `rootroot`).


## Setup
When using the face detection, please copy the `ros2_ws/src/sensing/sensing/haarcascade_frontalface_default.xml` file to `ros2_ws/install/sensing/lib/python3.10/site-packages/sensing/haarcascade_frontalface_default.xml`.
This is a workaround until a fix has been found.


## Servos

| Leg         | Joint | ID | Standing |
| ----------- | ----- | -- | :------: |
| Front Left  | Hip   | 05 | 140      |
| Front Left  | Upper | 02 | 130      |
| Front Left  | Lower | 00 | 070      |
| Front Right | Hip   | 04 | 105      |
| Front Right | Upper | 03 | 078      |
| Front Right | Lower | 01 | 132      |
| Back Left   | Hip   | 07 | 090      |
| Back Left   | Upper | 13 | 100      |
| Back Left   | Lower | 15 | 048      |
| Back Right  | Hip   | 06 | 110      |
| Back Right  | Upper | 12 | 179      |
| Back Right  | Lower | 14 | 110      |



## Documentation
Documentation of python scripts in this project should follow the [pep-287](https://peps.python.org/pep-0287/) standard.