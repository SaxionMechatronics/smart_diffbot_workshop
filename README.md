# SMART diffbot (ROS 2) workshop
This package is used to give ROS 2 workshops on several topics. It is inspired by our own [SMART diffbot package](https://github.com/SaxionMechatronics/smart_diffbot.git). The initial workshop starts with the end-result (`main` or `final`) branch to show and play around. Other lessons start with an initial state of this package, related to the topic of the workshop. These are captured in the following branches:

* `modeling`
* `control`
* `localization`
* `navigation`

The `final` branch contains a final result after completing all workshops. 

Tested for ROS 2 Jazzy and Gazebo Harmonic. 

Developed by the Smart Mechatronics And RoboTics (SMART) Research Group of Saxion University of Applied Sciences. This packages was in part supported by Regiorgaan SIA project RAAK-MKB Autonomous Agricultural Navigation (RAAK.MKB16.016). 

## Launch simulation

```bash
ros2 launch smart_diffbot_bringup main.launch.py 
```

## Start keyboard controller
```bash
ros2 run smart_diffbot_bringup keyboard_input 
```

## Monitor with RViz
```bash
ros2 launch smart_diffbot_control rviz.launch.py 
```
