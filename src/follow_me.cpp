#include "../include/follow_me/follow_me.h"

Follow::Follow(ros::NodeHandle *n) {
    printf("Start class of 'Follow'\n");
    this->ydlidar_sub = n->subscribe("/scan", 1, &Follow::ydlidar_callback, this);
    this->odom_sub = n->subscribe("/odom", 1000, &Follow::odom_callback, this);
    this->signal_sub = n->subscribe("/follow_me/control", 1000, &Follow::signal_callback, this);
    this->velocity_pub = n->advertise<rione_msgs::Velocity>("/move/velocity", 1000);
    this->twist_pub = n->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
    n->getParam("/follow_me/status", status);
}

Follow::~Follow() {
    printf("Shutdown class of 'Follow'\n");
}

void Follow::ydlidar_callback(const sensor_msgs::LaserScan::ConstPtr &msgs) {
    /*
     * ydlidarからの情報を取得
     */
    std::vector<cv::Point2d> ydlidar_points;
    double rad = msgs->angle_min;
    ydlidar_points.clear();
    ydlidar_ranges.clear();

    for (const auto &range : msgs->ranges) {
        cv::Point2d position;
        if (msgs->range_min + 0.05 < range && msgs->range_max > range) {
            position = cv::Point2d((int) (range * sin(rad) * 100), (int) (-range * cos(rad) * 100));
            ydlidar_points.push_back(position);
            if (min_distance > range) min_distance = range;
        } else {
            ydlidar_points.push_back(position);
        }
        ydlidar_ranges.push_back(range);
        rad += msgs->angle_increment;
    }

    if (data_list.empty()) {
        //初期化
        for (int i = 0; i < (int) ydlidar_points.size(); ++i) {
            //正規分布に従って初期化
            SampleData d = {i, ydlidar_points[i], calc_normal_distribution(i, 360, (int) ydlidar_points.size()) *
                                                  cost(cv::Point2d(0, 0), ydlidar_points[i])
            };
            data_list.push_back(d);
        }
    } else {
        //更新
        this->updatePlayerPoint(msgs->angle_increment, ydlidar_points);
        for (int i = 0; i < (int) ydlidar_points.size(); ++i) {
            data_list[i].point = ydlidar_points[i];
            data_list[i].existence_rate = cost(this->player_point, ydlidar_points[i]) +
                                          data_list[i].existence_rate *
                                          calc_normal_distribution(i, this->player_index, (int) ydlidar_points.size());
        }
    }
    //正規化
    double total = 0.0;
    double max = 0.0;
    int max_index = 0;

    for (int i = 0; i < (int) ydlidar_points.size(); ++i) {
        if (data_list[i].existence_rate == 0) data_list[i].existence_rate = 0.001;
        total += data_list[i].existence_rate;
        if (max < data_list[i].existence_rate) {
            max = data_list[i].existence_rate;
            max_index = i;
        }
    }
    for (int i = 0; i < (int) ydlidar_points.size(); ++i) {
        data_list[i].existence_rate *= 1 / total;
    }

    //制御
    //シグナルがtrueの時のみ実行
    if (status) {
        player_index = max_index;
        player_point = cv::Point2d(ydlidar_points[player_index]);
        rione_msgs::Velocity velocity;
        velocity.linear_rate = calcStraight(player_point);
        velocity.angular_rate = calcAngle(player_point);
        velocity_pub.publish(velocity);
    }

    this->view_ydlidar(ydlidar_points);
    std::cout << player_point << '\n';

}

void Follow::odom_callback(const nav_msgs::Odometry::ConstPtr &odom) {
    this->sensor_x = odom->pose.pose.position.x;
    this->sensor_y = odom->pose.pose.position.y;
    this->sensor_degree = Follow::toQuaternion_degree(odom->pose.pose.orientation.w, odom->pose.pose.orientation.z);
    this->sensor_rad = Follow::toQuaternion_rad(odom->pose.pose.orientation.w, odom->pose.pose.orientation.z);
}

double Follow::calc_normal_distribution(int target_index, int center_index, int index_size) {
    double index_distance = abs(target_index - center_index);
    if (index_distance > index_size / 2.0) index_distance -= index_size;
    index_distance *= 1 / 95.0;
    double normal_distribution = (1 / sqrt(2.0 * M_PI)) * exp((-index_distance * index_distance) / 2.0);
    return normal_distribution;
}

double Follow::calcAngle(const cv::Point2d &target_point) {
    double result = target_point.x * 0.008;
    result = result / 1.9;
    if (std::abs(result) > 1.0) result = 1.0;
    return result;
}

double Follow::calcStraight(const cv::Point2d &target_point) {
    /*
     * playerのy座標より進行方向の速度を計算
     * ただし、move_follow_flagによってしきい値を変更する
     */
    double result;
    if (std::abs((this->last_degree - this->sensor_degree)) > 40) return 0;
    if (target_point.y > 0) return 0;
    if (abs(target_point.y) > 100) {
        result = -target_point.y * 0.05;
    } else if (abs(target_point.y) < 80) {
        //result = (100 - abs(target_point.y)) * -0.004;
        result = 0;
    } else {
        result = 0;
    }
    result = result / 0.7;
    if (std::abs(result) > 1.0) result = 1.0;
    return result;
}

void Follow::updatePlayerPoint(double angle_increment, std::vector<cv::Point2d> ydlidar_points) {
    double relative_theta = Follow::toRadian(this->last_degree - this->sensor_degree);
    if (std::abs(relative_theta) > M_PI) {
        std::cout << "######################################################################" << '\n';
        relative_theta = (2 * M_PI) - relative_theta;
    }
    printf("relative_theta: %f\n", relative_theta);
    this->player_index += (int) (relative_theta / angle_increment);
    //if (!(ydlidar_points[this->player_index].x == 0 && ydlidar_points[this->player_index].y == 0))
    //    this->player_point = ydlidar_points[this->player_index];
    this->last_degree = this->sensor_degree;
}

void Follow::view_ydlidar(const std::vector<cv::Point2d> &points) {
    cv::Mat img = cv::Mat::zeros(1000, 1000, CV_8UC3);
    cv::Scalar color(0, 255, 0);
    for (auto &point : points) {
        int x = (int) point.x + 500;
        int y = (int) point.y + 500;
        cv::circle(img, cv::Point(x, y), 1, color, 1);
    }
    cv::circle(img, cv::Point(player_point.x + 500, player_point.y + 500), 5, cv::Scalar(0, 0, 255), 1);
    //cv::namedWindow("window", CV_WINDOW_NORMAL);
    cv::imshow("window", img);
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_me");
    ros::NodeHandle n;
    Follow follow(&n);
    ros::spin();
    return 0;
}
