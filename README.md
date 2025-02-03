# Independent Lunar Cave Mapping Module (ILUMO)

Simple overview of use/purpose.

## Description

An in-depth paragraph about your project and overview of use.

## Getting Started

### Dependencies

* 
* Linux 22.xx

### Installing

* Setup Jetson pins (If LED control/motor control is used)
* Setup Zed 2 camera/Blickfeld LiDAR/wavewshare thermal camera
* Clone repository
* install dependencies: TODO
* cd ilumo
* ```colcon build```


### Executing program

* Open new terminal
* . install/setup.bash TODO: not bash if other terminal is used
* ros2 launch ilumo_startup ilumo.launch.py
* or: ros2 run ilum_sensors lidar_publisher / stereo_camera_publisher / thermal_camera_publisher.py
* or: ros2 run ilumo_control pwm_controller.py
* Service calls for motor and LED
* Enjoy cool robot

## Help

Any advise for common problems or issues.
```
command to run if program contains helper info
```

## Author

* Malik-Manel Hashim (they/them)
* TU Berlin mail: m.hashim@campus.tu-berlin.de
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