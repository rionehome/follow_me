//
// Created by migly on 19/07/13.
//
// Modificated by ItoMasaki on 19/08/24
//


#include <iostream>
#include <cmath>

#include "ExtendedKalmanFilter/ExtendedKalmanFilter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>


#define MAX_LINEAR 0.7 // m/s
#define MAX_ANGULAR 1.9 // rad

using namespace std;

class Follow_me : public rclcpp::Node {
    private :
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Velocity_Publisher;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr Ydlidar_Subscription;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Signal_Subscription;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odometry_Subscription;

        void Ydlidar_Callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
        void Odometry_Callback(nav_msgs::msg::Odometry::SharedPtr msg);
        void Signal_Callback(std_msgs::msg::String::SharedPtr msg);

        void view_ydlidar(const std::vector<cv::Point> &points);
        double Quaternion2degree(double w, double z);
        double toAngle(double rad);
        double calcAngle(cv::Point target_point);
        double toRadian(double angle);
        double calcStraight(const cv::Point &target_point);

        ExtendedKalmanFilter *ekf;

        int i;
        bool status = true;

        vector<cv::Point> ydlidar_points;
        vector<double> ydlidar_ranges;

        double rad;
        double angle_increment;
        double sensor_degree = 0;
        double last_degree = 0;
        int min = 20;

        double distance;
        int min_index;
        double min_distance = DBL_MAX;
        cv::Point player_point{0, 0};

        rmw_qos_profile_t SENSOR_QOS_PROFILE = rmw_qos_profile_default;
        rmw_qos_profile_t PARAMETER_QOS_PROFILE = rmw_qos_profile_parameters;

    public : Follow_me() :
        Node("follow_me"){
            RCLCPP_INFO(this->get_logger(), "START FOLLOW ME");

            Ydlidar_Subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan",
                [this](sensor_msgs::msg::LaserScan::SharedPtr msg){
                    Ydlidar_Callback(msg);
                },
                SENSOR_QOS_PROFILE
            );

            Signal_Subscription = this->create_subscription<std_msgs::msg::String>(
                "signal",
                [this](std_msgs::msg::String::SharedPtr msg){
                    Signal_Callback(msg);
                },
                SENSOR_QOS_PROFILE
            );

            Odometry_Subscription = this->create_subscription<nav_msgs::msg::Odometry>(
                "/turtlebot2/odometry",
                [this](nav_msgs::msg::Odometry::SharedPtr msg){
                    Odometry_Callback(msg);
                },
                SENSOR_QOS_PROFILE
            );

            Velocity_Publisher = this->create_publisher<geometry_msgs::msg::Twist>(
                "/turtlebot2/commands/velocity",
                SENSOR_QOS_PROFILE
            );

        }

};
