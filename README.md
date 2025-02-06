# follow_me (Python3)
## *Disclaimer*
This package needs a massive rework!; otherwise it doesn't work correctly and is far from optimized.

## Description 
This is a Python3 implementation of ROS2 follow_me package (https://github.com/rionehome/follow_me/tree/ros2/master) which was originally written in C++.<br>
The content basically remains the same, but some modification and optimization has been done to make the code more readable and organized.<br>

## Publisher
- /cmd_vel          (geometry_msgs.msg/Twist)

## Subscriber
- /scan             (sensor_msgs.msg/LaserScan)
- /odometry         (nav_msgs.msg/Odometry)
- /sig_follow_me    (std_msgs.msg/String["start", "stop"])
