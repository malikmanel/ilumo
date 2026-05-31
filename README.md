# Independent Lunar Cave Mapping Module (ILUMO)

The ILUMO library contains ROS2 code for usage in the Independent Lunar Cave Mapping Module (ILUMO), a small robotic platform developed as part of my Master Thesis.

## Description

The ILUMO library consists of several ROS nodes for controlling the ILUMO robotic platform. Its main feature are the control nodes for sensorics (```ilumo_sensors```), specifically for:

* Zed 2 Stereo Camera
* Waveshare Long-wave IR Thermal Imaging Camera (USB version)
* Blickfeld Cube 1 LiDAR Camera

The relevant packages for control of the sensorics are directly incorporated into the ILUMO library, allowing for precise and low-level adjustments as needed. As the LiDAR and stereo camera control is written in C++, real-time processing becomes easier to achieve, especially as both nodes have parallel processing for image data and internal sensor data (IMU, thermometer, etc.) enabled. This becomes uniquely relevant for the LiDAR, as the node is set up to handel two LiDAR cameras simultaneously.

The library also includes a node for PWM control using the pins on the Jetson Orin Nano Developer Kit (```ilumo_control```).  It is assumed that signals are being sent to a motor controller board using 4 PWM pins as input to control a motor and LED lights.  Although the LED lights can be replaced by another device, the Jetson Orin Nano only contains 3 PWM pins, meaning the second device can only be operated in a single direction.

Furthermore, the library contains a urdf description of ILUMO as well as basic internal transforms (```ilumo_description```), custom messages and services (```ilumo_interfaces```), and a config and launch setup (```ilumo_startup```).


## Getting Started

### Dependencies

* Ubuntu 22.04

### Installing

Pre-installation:
* If motor/LED control is used: Setup Jetson pins (TODO: LINK)
* If LiDAR is used: Ensure the static IP matches the IPs defined in the LiDAR control node

ROS2 Setup:
* TODO

Installation:
* Clone repository
* install dependencies: TODO
* ```cd ilumo```
* ```colcon build```


### Executing program

In new terminal:
* ```. install/setup.bash```
* Full startup: ```ros2 launch ilumo_startup ilumo.launch.py```
* Single sensor: ```ros2 run ilum_sensors lidar_publisher / stereo_camera_publisher / thermal_camera_publisher.py```
* PWM control: ```ros2 run ilumo_control pwm_controller.py```
* ILUMO transforms: ```ros2 run ilumo_description ilumo_kinematics```
* TODO: Service calls for motor and LED

* Sensor and camera output can be visualized using ```ros2 run rqt_gui rqt_gui```
* LiDAR data and, ILUMO transforms and meshes can be visualized using ```ros2 run rviz2 rviz2```

## Help

Any advise for common problems or issues.
```
command to run if program contains helper info
```

## Author

* Malik-Manel Hashim (they/them)
* TU Berlin mail: m.hashim@campus.tu-berlin.de (Not available after 03/2025)
* [ResearchGate](https://www.researchgate.net/profile/Malik-Manel-Hashim)

## Version History

* 0.1
    * Initial Release

## License

This project is licensed under the [NAME HERE] License - see the LICENSE.md file for details

## Acknowledgments

* [Blickfeld Scanner Library](https://github.com/Blickfeld/blickfeld-scanner-lib)
* [Zed Open Capture](https://github.com/stereolabs/zed-open-capture)
* [Senxor](https://files.waveshare.com/wiki/Thermal-Camera-HAT/Thermal_camera_code.zip)
* [Jetson PWM control](https://forums.developer.nvidia.com/t/pwm-not-functioning-on-pins-32-and-33/283373)
* [Simple ReadMe](https://gist.github.com/DomPizzie/7a5ff55ffa9081f2de27c315f5018afc)