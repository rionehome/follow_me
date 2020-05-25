#include "follow_me/follow_me.hpp"

using namespace std;

void Follow_me::Ydlidar_Callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
    /*
     * ydlidarからの情報を取得
     */
    rad = msg->angle_min;
    angle_increment = msg->angle_increment;

    /*
     * clear points
     */
    ydlidar_points.clear();
    ydlidar_ranges.clear();

    /*
     * 
     */
    for (auto range : msg->ranges) {
        cv::Point position;
        int x = range * sin(rad) * 100;
        int y = -range * cos(rad) * 100;
        if ((-min<x && x<min) && (-min<y && y<min)) {
            x = DBL_MAX;
        }
        position = cv::Point(x, y);
        ydlidar_points.push_back(position);
        ydlidar_ranges.push_back(range);
        rad += angle_increment;
    }


    /*
     *
     */
    if (player_point.x == 0 && player_point.y == 0) {
        for (i = 0; i < (int) ydlidar_points.size(); ++i) {
            distance = sqrt(pow(ydlidar_points[i].x - player_point.x, 2) + pow(ydlidar_points[i].y - player_point.y, 2));
            if (distance < min_distance) {
                min_distance = distance;
                min_index = i;
            }
        }
        player_point = cv::Point(ydlidar_points[min_index]);
        ekf = new ExtendedKalmanFilter(player_point.x, player_point.y, 0.02);
    } else {
        min_distance = DBL_MAX;
        for (i = 0; i < (int) ydlidar_points.size(); ++i) {
            distance = sqrt(pow(ydlidar_points[i].x - player_point.x, 2) + pow(ydlidar_points[i].y - player_point.y, 2));
            if (distance < min_distance) {
                min_distance = distance;
                min_index = i;
            }
        }

        tuple<double, double> point;

        double dx = ydlidar_points[i].x - player_point.x;
        double dy = ydlidar_points[i].y - player_point.y;

        point = ekf->kalman_filter(ydlidar_points[min_index].x, ydlidar_points[min_index].y, dx, dy);

        player_point = cv::Point(get<0>(point), get<1>(point));
    }

    if (view_laser) {
        view_ydlidar(ydlidar_points);
    };

    //制御
    //シグナルがtrueの時のみ実行
    if (status) {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = calcStraight(player_point)/2;

        if (player_point.y > 0) {
            if (player_point.x > 0) {
                twist.angular.z = sqrt(pow(player_point.x, 2) + pow(player_point.y, 2))*0.01;
            } else {
                twist.angular.z = -sqrt(pow(player_point.x, 2) + pow(player_point.y, 2))*0.01;
            }
            
        } else {
            twist.angular.z = calcAngle(player_point);
        }

        Velocity_Publisher->publish(twist);
    }
}


void Follow_me::Odometry_Callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    this->sensor_degree = this->Quaternion2degree(msg->pose.pose.orientation.w, msg->pose.pose.orientation.z);
}


void Follow_me::Signal_Callback(rione_msgs::msg::Command::SharedPtr msg) {
    if (msg->command.data == "START") { // if recieve START command
        player_point.x = 0;
        player_point.y = 0;

        status = true;

        RCLCPP_INFO(this->get_logger(), "START");

    } else if(msg->command.data == "STOP"){
        status = false;
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        Velocity_Publisher->publish(twist);

        RCLCPP_INFO(this->get_logger(), "STOP FOLLOW ME");
    } else {
        RCLCPP_INFO(this->get_logger(), "UNKNOWN MESSAGE");
    }
}

double Follow_me::Quaternion2degree(double w, double z) {
    return std::abs((z > 0 ? 1 : 360) - toAngle(acos(w) * 2));
}

double Follow_me::toAngle(double rad) {
    return rad * 180 / M_PI;
}

double Follow_me::toRadian(double angle) { 
    return (angle * M_PI) / 180;
}

double Follow_me::calcAngle(cv::Point target_point) {
    double result = target_point.x * 0.01;
    return result;
}

double Follow_me::calcStraight(const cv::Point &target_point) {
    double result;
    if (target_point.y + 50 < 0) {
        result = -(target_point.y + 50) * 0.01;
    } else {
        result = 0;
    }
    return result;
}

void Follow_me::view_ydlidar(const std::vector<cv::Point> &points) {
    cv::Mat img = cv::Mat::zeros(1000, 1000, CV_8UC3);
    cv::Scalar color(0, 255, 0);
    for (auto &point : points) {
        int x = point.x + 500;
        int y = point.y + 250;
        cv::circle(img, cv::Point(x, y), 1, color, 1);
    }
    cv::circle(img, cv::Point(player_point.x + 500, player_point.y + 250), 5, cv::Scalar(0, 0, 255), 1);
    cv::namedWindow("window", CV_RAND_NORMAL);
    cv::imshow("window", img);
    cv::waitKey(1);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = make_shared<Follow_me>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
