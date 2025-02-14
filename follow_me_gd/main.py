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

from follow_me.util import Point, PointData

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
        self.sub_odm = self.create_subscription(Odometry, "/odometry", self.odometry_callback, 10)
        self.sub_cmd = self.create_subscription(String, "/follow_me/cmd", self.signal_callback, 10)

        ## User configurable ##
        self.view_laser:  bool = False  # whether to show graphical lidar data
        self.min_range:  float = 0.2    # min detection range [m]
        #######################

        self.status:      bool = True

        self.sensor_x:          float = 0.0
        self.sensor_y:          float = 0.0
        self.sensor_rad:        float = 0.0
        self.sensor_degree:     float = 0.0
        self.last_degree:       float = 0.0

        self.player_index:             int = -1
        self.player_point:           Point = Point(0, 0)
        self.last_absolute_position: Point = Point(0, 0)
        
        self.ydlidar_ranges: list[float] = []
        self.data_list:  list[PointData] = []

        self.get_logger().info(f"[+][follow_me] Successfully initialized.")


    def ydlidar_callback(self, msg) -> None:
        rad: float = msg.angle_min
        range_min: float = self.min_range if (self.min_range > msg.range_min) else msg.range_min

        ydlidar_points:      list[Point] = []
        self.ydlidar_ranges: list[float] = []
            
        # convert polar coordinate -> cartesian coordiante
        for measured_range in msg.ranges: # range measured in [m]
            position: Point = Point(0, 0)
            if (range_min <= measured_range and measured_range <= msg.range_max):
                position.x, position.y = (measured_range * np.sin(rad) * 100), (-measured_range * np.cos(rad) * 100)  # [m] -> [cm]
            ydlidar_points.append(position)
            self.ydlidar_ranges.append(measured_range)
            rad += msg.angle_increment

        # update PointData list
        if (len(self.data_list) == 0):  # init Gaussian distrib
            for i in range(len(ydlidar_points)):
                data: PointData = PointData(
                    index = i,
                    point = ydlidar_points[i],
                    existence_rate = self.calc_normal_distribution(i, 360, int(len(ydlidar_points)) * ydlidar_points[i].distance())
                )
                self.data_list.append(data)
        else:
            self.updatePlayerPoint(msg.angle_increment, ydlidar_points)
            for i in range(len(ydlidar_points)):
                try:
                    self.data_list[i].point = ydlidar_points[i]
                    self.data_list[i].existence_rate = self.cost(self.player_point, ydlidar_points[i]) + \
                                                       self.data_list[i].existence_rate * \
                                                       self.calc_normal_distribution(i, self.player_index, len(ydlidar_points))
                except IndexError as e:
                    break

        # normalization
        total:      float = 0.0
        max_val:    float = 0.0
        max_index:    int = 0
        for i in range(len(ydlidar_points)):
            try:
                if (self.data_list[i].existence_rate == 0): self.data_list[i].existence_rate = 0.001
                total += self.data_list[i].existence_rate
                if (max_val < self.data_list[i].existence_rate):
                    max_val     = self.data_list[i].existence_rate
                    max_index   = i
            except IndexError as e:
                break
        for i in range(len(ydlidar_points)):
            try:
                self.data_list[i].existence_rate /= total
            except IndexError as e:
                break

        # if enabled, publish twist according to estimated target's position
        if (self.status):
            self.player_index = max_index
            self.player_point = ydlidar_points[self.player_index]
            
            twist: Twist = Twist()
            twist.linear.x  = self.calcStraight(self.player_point)
            twist.angular.z = self.calcAngle(self.player_point) 
            self.pub_vel.publish(twist)

        new_position: Point = self.transform_absolute_to_relative(self.player_point)
        tmp: Point = Point(new_position.x - self.last_absolute_position.x,
                           new_position.y - self.last_absolute_position.y)
        self.get_logger().info(f"[+][follow_me] Distance: {tmp.distance()}")
        self.last_absolute_position = new_position

        # if enabled, visualize lidar data
        if (self.view_laser): self.view_ydlidar(ydlidar_points) 
        return


    def odometry_callback(self, odom) -> None:
        self.sensor_x = odom.pose.pose.position.x
        self.sensor_y = odom.pose.pose.position.y
        self.sensor_degree = self.quaternionToDegree(odom.pose.pose.orientation.w, odom.pose.pose.orientation.z)
        self.sensor_rad    = self.quaternionToRadian(odom.pose.pose.orientation.w, odom.pose.pose.orientation.z)
        return


    def signal_callback(self, msg) -> None:
        if (msg.data == "start" and self.status == False): self.status = True
        elif (msg.data == "stop" and self.status == True): self.status = False
        else: self.get_logger().info(f"[-][follow_me] Bad signal received: {msg.data}")

        if (self.status): self.get_logger().info(f"[+][follow_me] Activated!")
        else:
            twist: Twist = Twist()
            twist.linear.x  = 0
            twist.angular.z = 0
            self.pub_vel.publish(twist)
            self.get_logger().info(f"[+][follow_me] Deactivated!")
        return


    def calc_normal_distribution(self, target_index: int, center_index: int, index_size: int) -> float:
        index_distance: float = np.abs(target_index - center_index)
        if (index_distance > index_size / 2.0): index_distance -= index_size
        index_distance /= 95.0
        normal_distribution: float = 1.0 / np.sqrt(2.0 * np.pi) * np.exp((-index_distance * index_distance) / 2.0)
        return normal_distribution


    def transform_absolute_to_relative(self, relative_point: Point) -> Point:
        relative_theta: float = self.sensor_rad
        relative_x:     float = relative_point.x
        relative_y:     float = relative_point.y

        x: float = (relative_x * np.cos(relative_theta) - relative_y * np.sin(relative_theta)) + self.sensor_x
        y: float = (relative_x * np.sin(relative_theta) + relative_y * np.cos(relative_theta)) + self.sensor_y
        return Point(x, y)


    @staticmethod
    def cost(point1: Point, point2: Point) -> float:
        result: float = 0.01 if (point2.x == 0 and point2.y == 0) else Point.hypot(point1, point2)
        if (result >= sys.float_info.max): result = 0.1
        return result


    @staticmethod
    def quaternionToRadian(w: float, z: float) -> float:
        return 2 * np.arccos(w) * (-1 if (z < 0) else 1)


    @staticmethod
    def quaternionToDegree(w: float, z: float) -> float:
        return np.abs((1 if (z > 0) else 360) - self.toDegree(2 * np.arccos(w)))
    

    @staticmethod
    def toDegree(rad: float) -> float:
        return rad * 180 / np.pi


    @staticmethod
    def toRadian(angle: float) -> float:
        return (angle * np.pi) / 180


    @staticmethod
    def calcAngle(target_point: Point) -> float:
        result: float = target_point.x * 0.021
        if (np.abs(result) > MAX_ANGULAR): result = np.sign(result) * MAX_ANGULAR
        return result


    @staticmethod
    def calcStraight(target_point: Point) -> float:
        result: float = 0.0
        if (target_point.y < -70):
            result = np.abs(target_point.y) * 0.001875
        if (np.abs(result) > MAX_LINEAR): result = MAX_LINEAR
        return result


    def updatePlayerPoint(self, angle_increment: float, ydlidar_points: list[Point]) -> None:
        relative_theta: float = self.toRadian(self.last_degree - self.sensor_degree)
        if (np.abs(relative_theta) > np.pi): 
            relative_theta = (2 * np.pi - relative_theta)

        self.player_index += int(relative_theta / angle_increment)
        self.last_degree = self.sensor_degree
        return


    def view_ydlidar(self, points: list[Point]) -> None:
        """optional"""
        img: np.ndarray = np.zeros((1000, 1000, 3), dtype=np.uint8) 
        for point in points:
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
