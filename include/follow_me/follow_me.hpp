//
// Created by migly on 19/07/13.
//
// Modification by ItoMasaki on 19/08/24
//


//#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "sensor_msgs/Image.h"
//#include <sensor_msgs/LaserScan.h>
//#include <geometry_msgs/Twist.h>
//#include <cv_bridge/cv_bridge.h>  not to use
//#include <opencv2/highgui.hpp>    not to use
//#include <std_msgs/Float64MultiArray.h>
//#include <math.h>
//#include <iostream>
//#include <cmath>
//#include "move/Velocity.h"
//#include <nav_msgs/Odometry.h>
//#include <float.h>
//
//#ifndef SRC_FOLLOW_H_H
//#define SRC_FOLLOW_H_H

#include <iostream>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/String.hpp>
#include <sensor_msgs/msg/Image.hpp>
#include <sensor_msgs/msg/LaserScan.hpp>
#include <geometry_msgs/msg/Twist.hpp>
#include <nav_msgs/msg/Odometry.hpp>

#define MAX_LINEAR 0.7 // m/s
#define MAX_ANGULAR 1.9 // rad

class Follow_me: public rclcpp::Node {
	private :
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Velocity_Publisher;
		rclcpp::Subscriber<sensor_msgs::msg::LaserScan>::SharedPtr Ydlidar_Subscription;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Signal_Subscription;
		rclcpp::Subscription<geometry_msgs::msg::Odometry>::SharedPtr Odometry_Subscription;

		void Ydlidar_Callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
		void Odometry_Callback(nav_msgs::msg::Odometry::SharedPtr msg);
		void Signal_Callback(std_msgs::msg::String::SharedPtr msg);

		rmw_qos_profile_t SENSOR_QOS_PROFILE = rmw_qos_profile_sensor_data;
		rmw_qos_profile_t PARAMETER_QOS_PROFILE = rmw_qos_profile_parameters;

	public :
		Follow_me() :
			Node("follow_me"){
				Ydlidar_Subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
					"scan",
					[this](sensor_msgs::msg::LaserScan::SharedPtr msg){
						Ydlidar_Callback(msg);
					},
					SENSOR_QOS_PROFILE
				);

				Signal_Subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
					"signal",
					[this](sensor_msgs::msg::LaserScan::SharedPtr msg){
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
				)
			}

}

///////////////////////////////////////////////

    std_msgs::Float64MultiArray info;
    std::vector<double> ydlidar_ranges;
    std::vector<SampleData> data_list;

    int player_index = -1;
    double min_distance = DBL_MAX;
    double sensor_degree = 0;
    double last_degree = 0;
    bool status = false;
    bool move_follow_flag = false;
    cv::Point player_point;

    static double calc_normal_distribution(int target_index, int center_index, int index_size);

    static double cost(const cv::Point &p1, const cv::Point &p2)
    {
        double result = p2.x == 0 && p2.y == 0 ? 0.01 : 3 / hypot(p2.x - p1.x, p2.y - p1.y);
        if (std::isinf(result)) return 0.1;
        return result;
    }

    void view_ydlidar(const std::vector<cv::Point> &points);

    static double calcAngle(const cv::Point &target_point);

    static double calcStraight(const cv::Point &target_point);

    void signal_callback(const std_msgs::String::ConstPtr &msgs)
    {
        std::cout << msgs->data << '\n';
        status = msgs->data == "start";
        if (!status) {
            move::Velocity velocity;
            velocity.linear_rate = 0;
            velocity.angular_rate = 0;
            velocity_pub.publish(velocity);
        }
        else {
            printf("開始\n");
            data_list.clear();
        }
    }

    void updatePlayerPoint(const sensor_msgs::LaserScan_<std::allocator<void>>::ConstPtr &msgs);

    double toQuaternion_degree(double w, double z)
    {
        return std::abs((z > 0 ? 1 : 360) - this->toAngle(acos(w) * 2));
    }

    static double toQuaternion_rad(double w, double z)
    {
        return acos(w) * (z > 0 ? 1 : -1) * 2;
    }

    static double toAngle(double rad)
    { return rad * 180 / M_PI; }

    static double toRadian(double angle)
    { return (angle * M_PI) / 180; }

    static double sign(double A)
    { return A == 0 ? 0 : A / std::abs(A); }

    void publishTwist(double liner_x, double angular_z)
    {
        geometry_msgs::Twist twist = geometry_msgs::Twist();
        if (std::abs(liner_x) > MAX_LINEAR) liner_x = MAX_LINEAR * (std::signbit(liner_x) ? -1 : 1);
        twist.linear.x = liner_x;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        if (std::abs(angular_z) > MAX_ANGULAR) angular_z = MAX_ANGULAR * (std::signbit(angular_z) ? -1 : 1);
        twist.angular.z = angular_z;
        this->twist_pub.publish(twist);
    }
};

//#endif //SRC_FOLLOW_H_H
