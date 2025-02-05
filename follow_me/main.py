import sys
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from util import Point
from xkf import ExtendedKalmanFilter

# Turtlebot3 Waffle Specs
MAX_LINEAR  = 0.26 # m/s
MAX_ANGULAR = 1.82 # rad/s


class FollowMe(Node):

    def __init__(self):
        super().__init__("follow_me")
        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 10) 
        self.sub_ydl = self.create_subscription(LaserScan, "/scan", self.ydlidar_callback, 10)
        self.sub_odm = self.create_subscription(Odometry, "/odometry", self.odometry_callback, 10)
        self.sub_sig = self.create_subscription(String, "/sig_follow_me", self.signal_callback, 10)
        
        self.xkf: ExtendedKalmanFilter = None

        self.i:            int = 0
        self.status:      bool = False
        self.view_laser:  bool = False

        self.ydlidar_points: list[Point] = []
        self.ydlidar_ranges: list[float] = []

        self.rad:               float = 0
        self.angle_increment:   float = 0
        self.sensor_degree:     float = 0
        self.last_degree:       float = 0
        self.min:                 int = 20

        self.distance:      float = 0
        self.min_index:       int = 0
        self.min_distance:  float = sys.float_info.max
        self.player_point:  Point = Point(0, 0)

        self.get_logger().info(f"[+][follow_me] Successfully initialized.")


    def ydlidar_callback(self, msg) -> None:
        self.rad = msg.angle_min
        self.angle_increment = msg.angle_increment

        self.ydlidar_points = []
        self.ydlidar_ranges = []

        for range: float in msg.ranges: # range measured in [m]
            # convert polar coordinate -> cartesian coordiante
            x: int = int( range * np.sin(self.rad) * 100) # [m] -> [cm]
            y: int = int(-range * np.cos(self.rad) * 100) # [m] -> [cm]
            if (-1 * self.min < x && x < self.min) && (-1 * self.min < y && y < self.min): # change here to prevent detection of robot's parts
                x = sys.maxsize 
            position: Point = Point(x, y)
            self.ydlidar_points.append(position)
            self.ydlidar_ranges.append(range)
            self.rad += self.angle_increment

        if (self.player_point.x == 0 && self.player_point.y == 0):
            for i in range(len(self.ydlidar_points)):
                self.distance = np.sqrt(np.pow(self.ydlidar_points[i].x - self.player_point.x, 2) + np.pow(self.ydlidar_points[i].y - self.player_point.y, 2));
                if (self.distance < self.min_distance):
                    self.min_distance = distance
                    self.min_index = i
            self.player_point = self.ydlidar_points[self.min_index]
            self.xkf = ExtendedKalmanFilter(self.player_point.x, self.player_point.y, 0.02)
        else:
            self.min_distance = sys.float_info.max 
            for i in range(len(self.ydlidar_points)):
                self.distance = np.sqrt(np.pow(self.ydlidar_points[i].x - self.player_point.x, 2) + np.pow(self.ydlidar_points[i].y - self.player_point.y), 2)
                if (self.distance < self.min_distance):
                    self.min_distance = distance
                    self.min_index = i
            dx: float = self.ydlidar_points[i].x - self.player_point.x
            dy: float = self.ydlidar_points[i].y - self.player_point.y
            point: tuple[float, float] = self.xkf.kalman_filter(self.ydlidar_points[self.min_index].x, self.ydlidar_points[self.min_index].y, dx, dy)

            self.player_point = Point(point[0], point[1])

        if (view_laser): self.view_ydlidar() 

        if (status):
            twist: Twist = Twist()
            twist.linear.x = self.calcStraight() / 2

            if (self.player_point.y > 0):
                if (self.player_point.x > 0):
                    twist.angular.z = self.player_point.distance() * 0.01
                else:
                    twist.angular.z = self.player_point.distance() * -0.01
            else:
                twist.angular.z = self.calcAngle(self.player_point)


            self.pub_vel.publish(twist)
        return


    def odometry_callback(self, msg) -> None:
        self.sensor_degree = self.quaternionToDegree(msg.pose.pose.orientation.w, msg.pose.pose.orientation.z)
        return


    def signal_callback(self, msg) -> None:
        if (msg.data == "start"):
            self.status = True
        elif (msg.data == "stop"):
            self.status = False
        return


    def quaternionToDegree(self, w: float, z: float) -> float:
        return np.abs((1 if z > 0 else 360) - self.toAngle(np.arccos(w) * 2))  


    def toAngle(self, rad: float) -> float:
        return rad * 180 / np.pi()


    def toRadian(self, angle: float) -> float:
        return (angle * np.pi()) / 180


    def calcAngle(self, target_point: Point) -> float:
        return target_point.x * 0.01


    def calcStraight(self, target_point: Point) -> float:
        # [cm] -> [m]
        return -(target_point.y + 50) * 0.01 if target_point.y + 50 < 0 else 0


    def view_ydlidar(self) -> None:
        """optional"""
        img: np.ndarray = np.zero((1000, 1000, 3), dtype=np.uint8) 
        for point: Point in self.ydlidar_points:
            x = point.x + 500
            y = point.y + 250
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
