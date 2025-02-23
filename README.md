# follow_me (Python3)
## *Disclaimer*
This package adopts follow_me_gd (Gaussian Distribution) implementation.<br>
For Extended Kalman Filter(`follow_me_xkf`), please refer to ros2/dev branch.<br>

## Description 
This is a Python3 implementation of ROS2 follow_me package (https://github.com/rionehome/follow_me) which was originally written in C++.<br>
The content basically remains the same, but some modification and optimization has been done to make the code more readable and organized.<br>

## How to run
```bash
ros2 run follow_me follow_me
```

## Publisher
- /cmd_vel          (geometry_msgs.msg/Twist)

## Subscriber
- /scan             (sensor_msgs.msg/LaserScan)
- /odometry         (nav_msgs.msg/Odometry)
- /follow_me/cmd    (std_msgs.msg/String["start", "stop"])
