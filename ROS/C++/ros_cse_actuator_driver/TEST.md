# cse_actuator_driver
ROS nodelet for the PCA9685 PWM/Servo driver on the CS Enterprise.

## Build instructions
1. On the Raspberry Pi, ensure that `wiringPi`, `i2c-dev` and `Eigen3` are installed
```
sudo apt install wiringpi
sudo apt install libi2c-dev
sudo apt install libeigen3-dev
```

2. Build from workspace with ``catkin_make``


## Running the nodelet
```
rosrun nodelet nodelet manager __name:=nodelet_manager

rosrun nodelet nodelet load cse_actuator_driver/cse_actuator_driver_nodelet nodelet_manager __name:=cse_actuator_driver
```

## Topics
The nodelet subscribes to the joystick and control input topics `/joy` and `/CSEI/u`, respectively.

## Usage
 Pressing `circle` on the connected joystick disables all actuators, while `cross` converts the input vector
<p align="center"><img src="https://rawgit.com/NTNU-MCS/CS_EnterpriseI_archive/None/svgs/a3799cd070aa26e27e4a1d5102ae0e16.svg?invert_in_darkmode" align=middle width=279.25493969999997pt height=16.438356pt/></p>

from `/CSEI/u` into appropriate pwm signals.