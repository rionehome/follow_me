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

#ifndef SRC_FOLLOW_H_H
#define SRC_FOLLOW_H_H

class Follow {
public:
    Follow();

    ~Follow();

    //インデックスごとの構造体を作成
    typedef struct {
      int index;
      cv::Point point;
      double existence_rate;
    } SampleData;

    ros::Publisher move_pub;
    ros::Publisher output_pub;
    ros::Subscriber ydlider;
    ros::Subscriber posenet;
    ros::Subscriber signal;

    std_msgs::Float64MultiArray info;
    std::vector<cv::Point> ydlider_points;
    std::vector<double> ydlider_ranges;
    std::vector<cv::Point> posenet_points;
    std::vector<SampleData> data_list;

    int player_index = -1;
    bool status = false;
    bool move_follow_flag = false;
    cv::Point player_point;

    ros::NodeHandle n;

    void update();

    static double calc_normal_distribution(int target_index, int center_index, int index_size);

    static double cost(const cv::Point& p1, const cv::Point& p2)
    {
        double result = p2.x==0 && p2.y==0 ? 0.01 : 3/hypot(p2.x-p1.x, p2.y-p1.y);
        if (std::isinf(result)) return 0.1;
        return result;
    }

    void view_ydlider(const std::vector<cv::Point>& points);

    static double calcAngle(const cv::Point& target_point);

    double calcStraight(const cv::Point& target_point);

    void signal_callback(const std_msgs::String::ConstPtr& msgs)
    {
        std::cout << msgs->data << '\n';
        status = msgs->data=="start";
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

    void ydlider_callback(const sensor_msgs::LaserScan::ConstPtr& msgs);
};

#endif //SRC_FOLLOW_H_H
