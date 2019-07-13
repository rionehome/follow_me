//
// Created by migly on 19/07/13.
//
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include <iostream>
#include <cmath>
#include <follow_me/FollowOutput.h>
#include <nav_msgs/Odometry.h>

#ifndef SRC_FOLLOW_H_H
#define SRC_FOLLOW_H_H

class Follow
{
public:
    Follow();

    ~Follow();

    //インデックスごとの構造体を作成
    typedef struct
    {
        int index;
        cv::Point point;
        double existence_rate;
    } SampleData;

    ros::Publisher move_pub;
    ros::Publisher output_pub;
    ros::Subscriber ydlidar_sub;
    ros::Subscriber signal;
    ros::Subscriber odom_sub;

    std_msgs::Float64MultiArray info;
    std::vector<double> ydlidar_ranges;
    std::vector<SampleData> data_list;

    int player_index = -1;
    bool status = false;
    bool move_follow_flag = false;
    cv::Point player_point;

    ros::NodeHandle n;

    static double calc_normal_distribution(int target_index, int center_index, int index_size);

    static double cost(const cv::Point &p1, const cv::Point &p2)
    {
        double result = p2.x == 0 && p2.y == 0 ? 0.01 : 3 / hypot(p2.x - p1.x, p2.y - p1.y);
        if (std::isinf(result)) return 0.1;
        return result;
    }

    void view_ydlidar(const std::vector<cv::Point> &points);

    static double calcAngle(const cv::Point &target_point);

    double calcStraight(const cv::Point &target_point);

    void signal_callback(const std_msgs::String::ConstPtr &msgs)
    {
        std::cout << msgs->data << '\n';
        status = msgs->data == "start";
        if (!status) {
            info.data.clear();
            info.data.push_back(0);
            info.data.push_back(0);
            info.data.push_back(0);
            info.data.push_back(0);
            move_pub.publish(info);
        }
        else {
            data_list.clear();
        }
    }

    void ydlidar_callback(const sensor_msgs::LaserScan::ConstPtr &msgs);

    void odom_callback(const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void>>> &odom);

    double toQuaternion_ang(double w, double z)
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
};

#endif //SRC_FOLLOW_H_H
