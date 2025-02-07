# follow_me (Python3)
## *Disclaimer*
Now this package has 2 different types of follow_me implementations.<br>
One uses Extended Kalman Filter(`follow_me_xkf`) and another is based on Gaussian distribution(`follow_me_gd`).

## Description 
This is a Python3 implementation of ROS2 follow_me package (https://github.com/rionehome/follow_me) which was originally written in C++.<br>
The content basically remains the same, but some modification and optimization has been done to make the code more readable and organized.<br>

## Publisher
- /cmd_vel          (geometry_msgs.msg/Twist)

## Subscriber
- /scan             (sensor_msgs.msg/LaserScan)
- /odometry         (nav_msgs.msg/Odometry)
- /follow_me/cmd    (std_msgs.msg/String["start", "stop"])
