import sys
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from follow_me_xkf.util import Point
from follow_me_xkf.xkf import ExtendedKalmanFilter

# Turtlebot3 Waffle Specs
MAX_LINEAR  = 0.26 # m/s
MAX_ANGULAR = 1.82 # rad/s


class FollowMe(Node):

    def __init__(self):
        super().__init__("follow_me")
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # Make sure it's reliable
        )

        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 10) 
        self.sub_ydl = self.create_subscription(LaserScan, "/scan", self.ydlidar_callback, qos_profile=qos_profile)
        self.sub_cmd = self.create_subscription(String, "/follow_me/cmd", self.signal_callback, 10)

        ## User configurable ##
        self.view_laser:  bool = False  # whether to show graphical lidar data
        self.min_range:  float = 0.2    # min detection range [m]
        #######################

        self.status:                     bool = True
        self.player_point:              Point = Point(0, 0)
        self.ydlidar_points:      list[Point] = []
        self.xkf: ExtendedKalmanFilter        = None

        self.get_logger().info("Successfully initialized.")


    def ydlidar_callback(self, msg) -> None:
        rad: float = msg.angle_min
        range_min: float = self.min_range if (self.min_range > msg.range_min) else msg.range_min
        self.ydlidar_points = []
            
        # convert polar coordinate -> cartesian coordiante
        for measured_range in msg.ranges: # range measured in [m]
            if (range_min <= measured_range and measured_range <= msg.range_max):
                position: Point = Point(measured_range * np.sin(rad) * 100, -measured_range * np.cos(rad) * 100)  # [m] -> [cm]
                self.ydlidar_points.append(position)
            rad += msg.angle_increment

        # find closest object
        min_distance: float = sys.float_info.max 
        min_index:      int = 0
        for i in range(len(self.ydlidar_points)):
            distance: float = np.sqrt((self.ydlidar_points[i].x - self.player_point.x) ** 2 + (self.ydlidar_points[i].y - self.player_point.y) ** 2)
            if (distance < min_distance):
                min_distance = distance
                min_index = i
        
        # init xkf or update estimated target position
        if (self.player_point.x == 0 and self.player_point.y == 0):
            self.player_point = self.ydlidar_points[min_index]
            self.xkf = ExtendedKalmanFilter(self.player_point.x, self.player_point.y)
        else:
            dx: float = self.ydlidar_points[i].x - self.player_point.x
            dy: float = self.ydlidar_points[i].y - self.player_point.y
            self.player_point = self.xkf.kalman_filter(self.ydlidar_points[min_index].x, self.ydlidar_points[min_index].y, dx, dy)

        # if enabled, publish twist according to estimated target's position
        if (self.status):
            twist: Twist = Twist()
            twist.linear.x  = self.calcStraight()
            twist.angular.z = self.calcAngle() 
            self.pub_vel.publish(twist)

        # if enabled, visualize lidar data
        if (self.view_laser): self.view_ydlidar() 
        return


    def signal_callback(self, msg) -> None:
        if (msg.data == "start" and self.status == False): self.status = True
        elif (msg.data == "stop" and self.status == True): self.status = False
        else: self.get_logger().info(f"Bad signal received: {msg.data}")

        if (self.status): self.get_logger().info(f"Activated!")
        else:
            twist: Twist = Twist()
            twist.linear.x  = 0
            twist.angular.z = 0
            self.pub_vel.publish(twist)
            self.get_logger().info(f"Deactivated!")
        return


    def calcAngle(self) -> float:
        result = self.player_point.x * 0.021
        if (np.abs(result) > MAX_ANGULAR): result = MAX_ANGULAR
        return result


    def calcStraight(self) -> float:
        result = 0.0
        if (self.player_point.y < 0 and np.abs(self.player_point.y) > 70):
            result = np.abs(self.player_point.y) * 0.001875
        if (np.abs(result) > MAX_LINEAR): result = MAX_LINEAR
        return result


    def view_ydlidar(self) -> None:
        """optional"""
        img: np.ndarray = np.zeros((1000, 1000, 3), dtype=np.uint8) 
        for point in self.ydlidar_points:
            x = int(point.x) + 500
            y = int(point.y) + 250
            cv2.circle(img, center=(x, y), radius=1, color=(0, 255, 0), thickness=1)
        cv2.circle(img, center=(self.player_point.x + 500, self.player_point.y + 250), radius=5, color=(0, 0, 255), thickness=1)
        cv2.imshow("follow_me", img)
        cv2.waitKey(1)
        return


def main(args=None):
    rclpy.init(args=args)

    node = FollowMe()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    
    cv2.destroyWindow("follow_me")
    cv2.waitKey(1)


if __name__ == "__main__":
    main()
