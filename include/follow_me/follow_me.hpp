//
// Created by migly on 19/07/13.
//
// Modification by ItoMasaki on 19/08/24
//


//#include "move/Velocity.h"
//#ifndef SRC_FOLLOW_H_H
//#define SRC_FOLLOW_H_H

#include <iostream>
#include <cmath>

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

class Follow_me : public rclcpp::Node {
    private :
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Velocity_Publisher;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr Ydlidar_Subscription;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Signal_Subscription;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odometry_Subscription;

        void Ydlidar_Callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
        void Odometry_Callback(nav_msgs::msg::Odometry::SharedPtr msg);
        void Signal_Callback(std_msgs::msg::String::SharedPtr msg);

        void updatePlayerPoint();
        void view_ydlidar(const std::vector<cv::Point> &points);
        double toQuaternion_degree(double w, double z);
        double toAngle(double rad);
        double calcAngle(const cv::Point &target_point);
        double toRadian(double angle);
        double calcStraight(const cv::Point &target_point);


        int i;
        bool status = true;

        std::vector<cv::Point> ydlidar_points;
        std::vector<double> ydlidar_ranges;
        double rad;
        double angle_increment;
        double sensor_degree = 0;
        double last_degree = 0;

        double distance;
        int min_index;
        double min_distance = DBL_MAX;
        cv::Point player_point = {0, 0};

        rmw_qos_profile_t SENSOR_QOS_PROFILE = rmw_qos_profile_sensor_data;
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
                PARAMETER_QOS_PROFILE
            );

            Odometry_Subscription = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                [this](nav_msgs::msg::Odometry::SharedPtr msg){
                    Odometry_Callback(msg);
                },
                SENSOR_QOS_PROFILE
            );
        }

};

///////////////////////////////////////////////
//    std::vector<SampleData> data_list;
//
//    int player_index = -1;
//    bool move_follow_flag = false;
//
//    static double toQuaternion_rad(double w, double z)
//    {
//        return acos(w) * (z > 0 ? 1 : -1) * 2;
//    }
//
//    static double sign(double A)
//    { return A == 0 ? 0 : A / std::abs(A); }
//
//    void publishTwist(double liner_x, double angular_z)
//    {
//        geometry_msgs::Twist twist = geometry_msgs::Twist();
//        if (std::abs(liner_x) > MAX_LINEAR) liner_x = MAX_LINEAR * (std::signbit(liner_x) ? -1 : 1);
//        twist.linear.x = liner_x;
//        twist.linear.y = 0.0;
//        twist.linear.z = 0.0;
//        twist.angular.x = 0.0;
//        twist.angular.y = 0.0;
//        if (std::abs(angular_z) > MAX_ANGULAR) angular_z = MAX_ANGULAR * (std::signbit(angular_z) ? -1 : 1);
//        twist.angular.z = angular_z;
//        this->twist_pub.publish(twist);
//    }
//};

//#endif //SRC_FOLLOW_H_H
