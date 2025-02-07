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

        ## User configurable ##
        self.view_laser:  bool = False  # whether to show graphical lidar data
        self.min_range:  float = 0.2    # min detection range [m]
        #######################

        self.status:      bool = False

        self.sensor_x:          float = 0.0
        self.sensor_y:          float = 0.0
        self.sensor_rad:        float = 0.0
        self.sensor_degree:     float = 0.0
        self.last_degree:       float = 0.0

        self.min_index:       int = 0
        self.distance:      float = 0.0
        self.min_distance:  float = sys.float_info.max

        self.player_point:              Point = Point(0, 0)
        self.last_absolute_position:    Point = Point(0, 0)
        
        self.stack_absolute_positions:  list[Point] = []
        self.ydlidar_points:            list[Point] = []

        self.xkf: ExtendedKalmanFilter = None

        self.get_logger().info(f"[+][follow_me] Successfully initialized.")


    def ydlidar_callback(self, msg) -> None:
        rad: float = msg.angle_min
        range_min: float = self.min_range if (self.min_range > msg.range_min) else msg.range_min

        self.ydlidar_points = []
            
        # convert polar coordinate -> cartesian coordiante
        for range: float in msg.ranges: # range measured in [m]
            if (range_min <= range && range <= msg.range_max):
                position: Point = Point(range * np.sin(rad) * 100, -range * np.cos(rad) * 100)  # [m] -> [cm]
                self.ydlidar_points.append(position)
            rad += msg.angle_increment

        # find closest object
        self.min_distance = sys.float_info.max 
        for i in range(len(self.ydlidar_points)):
            self.distance = np.sqrt(np.pow(self.ydlidar_points[i].x - self.player_point.x, 2) + np.pow(self.ydlidar_points[i].y - self.player_point.y, 2));
            if (self.distance < self.min_distance):
                self.min_distance = distance
                self.min_index = i
        
        # init xkf or update estimated target position
        if (self.player_point.x == 0 && self.player_point.y == 0):
            self.player_point = self.ydlidar_points[self.min_index]
            self.xkf = ExtendedKalmanFilter(self.player_point.x, self.player_point.y, 0.02)
        else:
            dx: float = self.ydlidar_points[i].x - self.player_point.x
            dy: float = self.ydlidar_points[i].y - self.player_point.y
            self.player_point = self.xkf.kalman_filter(self.ydlidar_points[self.min_index].x, self.ydlidar_points[self.min_index].y, dx, dy)

        # if enabled, publish twist according to estimated target's position
        if (status):
            twist: Twist = Twist()
            twist.linear.x  = self.calcStraight()
            twist.angular.z = self.calcAngle() 
            self.pub_vel.publish(twist)

        # if enabled, visualize lidar data
        if (view_laser): self.view_ydlidar() 
        return


    def odometry_callback(self, msg) -> None:
        self.sensor_degree = self.quaternionToDegree(msg.pose.pose.orientation.w, msg.pose.pose.orientation.z)
        return


    def signal_callback(self, msg) -> None:
        if (msg.data == "start" && self.status == False): self.status = True
        elif (msg.data == "stop" && self.status == True): self.status = False
        else: self.get_logger().info(f"[-][follow_me] Bad signal received: {msg.data}")

        if (self.status): self.get_logger().info(f"[+][follow_me] Activated!")
        else:
            twist: Twist = Twist()
            twist.linear.x  = 0
            twist.angular.z = 0
            self.pub_vel.publish(twist)
            self.get_logger().info(f"[+][follow_me] Deactivated!")
        return


    def quaternionToDegree(self, w: float, z: float) -> float:
        return np.abs((1 if z > 0 else 360) - self.toAngle(np.arccos(w) * 2))  


    @staticmethod
    def toAngle(rad: float) -> float:
        return rad * 180 / np.pi()


    @staticmethod
    def toRadian(angle: float) -> float:
        return (angle * np.pi()) / 180


    def calcAngle(self) -> float:
        result = self.player_point.x * 0.021
        if (np.abs(result) > MAX_ANGULAR): result = MAX_ANGULAR
        return result


    def calcStraight(self) -> float:
        result = 0;
        if (self.player_point.y < 0 && np.abs(self.player_point.y) > 70):
            result = np.abs(self.player_point.y) * 0.001875
        return result


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
