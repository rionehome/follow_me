//
// Created by migly on 19/07/13.
//
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <iostream>
#include <cmath>
#include <rione_msgs/Velocity.h>
#include <nav_msgs/Odometry.h>
#include <cfloat>

#ifndef SRC_FOLLOW_H_H
#define SRC_FOLLOW_H_H
#define MAX_LINEAR 0.7 // m/s
#define MAX_ANGULAR 1.9 // rad

class Follow {
public:
    explicit Follow(ros::NodeHandle *n);

    ~Follow();

    //インデックスごとの構造体を作成
    typedef struct {
        int index;
        cv::Point2d point;
        double existence_rate;
    } SampleData;

    ros::Publisher velocity_pub;
    ros::Subscriber ydlidar_sub;
    ros::Subscriber signal_sub;
    ros::Subscriber odom_sub;
    ros::Publisher twist_pub;

    std_msgs::Float64MultiArray info;
    std::vector<double> ydlidar_ranges;
    std::vector<SampleData> data_list;

    int player_index = -1;
    double min_distance = DBL_MAX;
    double sensor_x = 0;
    double sensor_y = 0;
    double sensor_degree = 0;
    double sensor_rad = 0;
    double last_degree = 0;
    bool status = false;
    cv::Point2d player_point;

    static double calc_normal_distribution(int target_index, int center_index, int index_size);

    static double cost(const cv::Point2d &p1, const cv::Point2d &p2) {
        double result = p2.x == 0 && p2.y == 0 ? 0.01 : 3 / hypot(p2.x - p1.x, p2.y - p1.y);
        if (std::isinf(result)) return 0.1;
        return result;
    }

    void view_ydlidar(const std::vector<cv::Point2d> &points);

    static double calcAngle(const cv::Point2d &target_point);

    double calcStraight(const cv::Point2d &target_point);

    void signal_callback(const std_msgs::String::ConstPtr &msgs) {
        std::cout << msgs->data << '\n';
        status = msgs->data == "start";
        if (!status) {
            rione_msgs::Velocity velocity;
            velocity.linear_rate = 0;
            velocity.angular_rate = 0;
            velocity_pub.publish(velocity);
        } else {
            printf("開始\n");
            data_list.clear();
        }
    }

    void ydlidar_callback(const sensor_msgs::LaserScan::ConstPtr &msgs);

    void odom_callback(const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void>>> &odom);

    void transform_absolute_to_relative(cv::Point2d &relative_point) {

    }

    static double toQuaternion_degree(double w, double z) {
        return std::abs((z > 0 ? 1 : 360) - Follow::toAngle(acos(w) * 2));
    }

    static double toQuaternion_rad(double w, double z) {
        return acos(w) * 2 * (signbit(z) ? -1 : 1);
    }

    static double toAngle(double rad) { return rad * 180 / M_PI; }

    static double toRadian(double angle) { return (angle * M_PI) / 180; }

    void updatePlayerPoint(double angle_increment, std::vector<cv::Point2d> ydlidar_points);
};

#endif //SRC_FOLLOW_H_H
