#include "../include/follow_me/follow_me.hpp"
#include "../include/ExtendedKalmanFilter/ExtendedKalmanFilter.hpp"

using namespace std;

void Follow_me::Ydlidar_Callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
    /*
     * ydlidarからの情報を取得
     */
    rad = msg->angle_min;
    angle_increment = msg->angle_increment;
    ydlidar_points.clear();
    ydlidar_ranges.clear();

    for (auto range : msg->ranges) {
        cv::Point position;
        if (msg->range_min + 0.05 < range && msg->range_max > range) {
            position = cv::Point((int) (range * sin(rad) * 100), (int) (-range * cos(rad) * 100));
            ydlidar_points.push_back(position);
            if (min_distance > range) {
                min_distance = range;
            }
        } else {
            ydlidar_points.push_back(position);
        }
        ydlidar_ranges.push_back(range);
        rad += msg->angle_increment;
    }

    if (player_point.x == 0 && player_point.y == 0) {
        //初期化
        for (i = 0; i < (int) ydlidar_points.size(); ++i) {
            distance = sqrt(pow(ydlidar_points[i].x - player_point.x, 2) + pow(ydlidar_points[i].y - player_point.y, 2));
            if (distance < min_distance) {
                min_distance = distance;
                min_index = i;
            }
        }
        player_point.x = ydlidar_points[min_index].x;
        player_point.y = ydlidar_points[min_index].y;
        cout << player_point << endl;
    } else {
        //更新
        this->updatePlayerPoint();
    }

    this->view_ydlidar(ydlidar_points);

    //制御
    //シグナルがtrueの時のみ実行
    if (status) {
        player_point = cv::Point(player_point);
        cout << player_point.x << " : " << player_point.y << endl;
        /*
        follow_me::Output output = follow_me::Output();
        output.index = player_index;
        output.range = ydlidar_ranges[player_index];
        output_pub.publish(output);
        */
        //move::Velocity velocity;
        //velocity.linear_rate = calcStraight(player_point);
        //velocity.angular_rate = calcAngle(player_point);
        //velocity_pub.publish(velocity);
    }
}


void Follow_me::Odometry_Callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    this->sensor_degree = this->toQuaternion_degree(msg->pose.pose.orientation.w, msg->pose.pose.orientation.z);
}


void Follow_me::Signal_Callback(std_msgs::msg::String::SharedPtr msg) {
    std::cout << msg->data << '\n';
    status = msg->data == "start";
    if (!status) {
        //move::Velocity velocity;
        //velocity.linear_rate = 0;
        //velocity.angular_rate = 0;
        //velocity_pub.publish(velocity);
    } else {
        printf("開始\n");
    }
}

double Follow_me::toQuaternion_degree(double w, double z) {
    return std::abs((z > 0 ? 1 : 360) - toAngle(acos(w) * 2));
}

double Follow_me::toAngle(double rad) {
    return rad * 180 / M_PI;
}

double Follow_me::toRadian(double angle) { 
    return (angle * M_PI) / 180;
}

double Follow_me::calcAngle(const cv::Point &target_point)
{
    double result = target_point.x * 0.01;
    result = result / 1.9;
    printf("angular:%f\n", result);
    return result;
}

double Follow_me::calcStraight(const cv::Point &target_point)
{
    /*
     * playerのy座標より進行方向の速度を計算
     * ただし、move_follow_flagによってしきい値を変更する
     */
    double result;
    if (abs(target_point.y) > 100) {
        result = -target_point.y * 0.05;
    }
    else if (abs(target_point.y) < 80) {
        //result = (100 - abs(target_point.y)) * -0.004;
        result = 0;
    }
    else {
        result = 0;
    }
    if (result > 0.7) result = 0.7;
    result = result / 0.7;
    std::cout << result << '\n';
    return result;
}

void Follow_me::updatePlayerPoint() {
    double relative_theta = this->toRadian(this->last_degree - this->sensor_degree);
    double relative_x = this->player_point.x;
    double relative_y = this->player_point.y;

    //this->player_point.x = relative_x * cos(relative_theta) - relative_y * sin(relative_theta);
    //this->player_point.y = relative_x * sin(relative_theta) + relative_y * cos(relative_theta);
}

void Follow_me::view_ydlidar(const std::vector<cv::Point> &points)
{
    cv::Mat img = cv::Mat::zeros(1000, 1000, CV_8UC3);
    cv::Scalar color(0, 255, 0);
    for (auto &point : points) {
        int x = point.x + 1000;
        int y = point.y + 1000;
        cv::circle(img, cv::Point(x, y), 1, color, 1);
    }
    cv::circle(img, cv::Point(player_point.x + 1000, player_point.y + 1000), 5, cv::Scalar(0, 0, 255), 1);
    cv::namedWindow("window", CV_WINDOW_NORMAL);
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
