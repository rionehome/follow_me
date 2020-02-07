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
#include <rione_msgs/msg/command.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>


#define MAX_LINEAR 0.7 // m/s
#define MAX_ANGULAR 1.9 // rad

using namespace std;

class Follow_me : public rclcpp::Node {
    private :
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Velocity_Publisher;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr Ydlidar_Subscription;
        rclcpp::Subscription<rione_msgs::msg::Command>::SharedPtr Signal_Subscription;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odometry_Subscription;

        void Ydlidar_Callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
        void Odometry_Callback(nav_msgs::msg::Odometry::SharedPtr msg);
        void Signal_Callback(rione_msgs::msg::Command::SharedPtr msg);

        void view_ydlidar(const std::vector<cv::Point> &points);
        double Quaternion2degree(double w, double z);
        double toAngle(double rad);
        double calcAngle(cv::Point target_point);
        double toRadian(double angle);
        double calcStraight(const cv::Point &target_point);

        ExtendedKalmanFilter *ekf;

        int i;
        bool status = false;

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

        int view_laser;

    public : Follow_me() :
        Node("follow_me"){
            RCLCPP_INFO(this->get_logger(), "START FOLLOW ME");

            Ydlidar_Subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan",
                10,
                [this](sensor_msgs::msg::LaserScan::SharedPtr msg){
                    Ydlidar_Callback(msg);
                }
            );

            Signal_Subscription = this->create_subscription<rione_msgs::msg::Command>(
                "signal",
                10,
                [this](rione_msgs::msg::Command::SharedPtr msg){
                    Signal_Callback(msg);
                }
            );

            Odometry_Subscription = this->create_subscription<nav_msgs::msg::Odometry>(
                "/turtlebot2/odometry",
                10,
                [this](nav_msgs::msg::Odometry::SharedPtr msg){
                    Odometry_Callback(msg);
                }
            );

            Velocity_Publisher = this->create_publisher<geometry_msgs::msg::Twist>(
                "/turtlebot2/commands/velocity",
                10
            );

            declare_parameter("view_laser", 1);
            view_laser = this->get_parameter("view_laser").as_int();

        }

};
