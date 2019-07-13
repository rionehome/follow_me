#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "../include/followme/Follow_H.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include <iostream>
#include <cmath>
#include <follow_me/FollowOutput.h>

Follow::Follow()
{
    printf("Start class of 'Follow'\n");
    this->ydlider = n.subscribe("/scan", 1, &Follow::ydlider_callback, this);
    this->move_pub = n.advertise<std_msgs::Float64MultiArray>("/move/velocity", 1000);
    this->signal = n.subscribe("/follow_me/control", 1, &Follow::signal_callback, this);
    this->output_pub = n.advertise<follow_me::FollowOutput>("/follow_me/output", 1000);
}

Follow::~Follow()
{
    printf("Shutdown class of 'Follow'\n");
}

double Follow::calc_normal_distribution(int target_index, int center_index, int index_size)
{
    double index_distance = abs(target_index-center_index);
    if (index_distance>index_size/2) index_distance -= index_size;
    index_distance *= 1/95.0;
    double normal_distribution = (1/sqrt(2.0*M_PI))*exp((-index_distance*index_distance)/2.0);
    return normal_distribution;
}

double Follow::calcAngle(const cv::Point& target_point)
{
    double result = target_point.x*0.02;
    return result;
}

double Follow::calcStraight(const cv::Point& target_point)
{
    /*
     * playerのy座標より進行方向の速度を計算
     * ただし、move_follow_flagによってしきい値を変更する
     */
    double result;
    if (move_follow_flag) {
        result = abs(target_point.y)>120 ? -target_point.y*0.008 : 0;
    }
    else {
        result = abs(target_point.y)>120 ? -target_point.y*0.01 : 0;
    }
    move_follow_flag = true;
    if (result>0.7) result = 0.7;
    if (result<=0) {
        result = 0;
        move_follow_flag = false;
    }
    return result;
}

void Follow::update()
{
    if ((int) ydlider_points.size()==0) return;
    if (data_list.empty()) {
        //初期化
        for (int i = 0; i<(int) ydlider_points.size(); ++i) {
            //正規分布に従って初期化
            SampleData d = {i, ydlider_points[i], calc_normal_distribution(i, 360, (int) ydlider_points.size())*
                    cost(cv::Point(0, 0), ydlider_points[i])
            };
            data_list.push_back(d);
        }
    }
    else {
        //更新
        for (int i = 0; i<(int) ydlider_points.size(); ++i) {
            data_list[i].point = ydlider_points[i];
            data_list[i].existence_rate = cost(player_point, ydlider_points[i])+data_list[i].existence_rate*
                    calc_normal_distribution(i,
                            player_index,
                            (int) ydlider_points.size());
        }
    }
    //正規化
    double total = 0.0;
    double max = 0.0;
    int max_index = 0;

    for (int i = 0; i<(int) ydlider_points.size(); ++i) {
        if (data_list[i].existence_rate==0) data_list[i].existence_rate = 0.001;
        total += data_list[i].existence_rate;
        if (max<data_list[i].existence_rate) {
            max = data_list[i].existence_rate;
            max_index = i;
        }
    }
    for (int i = 0; i<(int) ydlider_points.size(); ++i) {
        data_list[i].existence_rate *= 1/total;
    }

    //制御
    //シグナルがtrueの時のみ実行
    if (status) {
        player_index = max_index;
        player_point = cv::Point(ydlider_points[player_index]);
        follow_me::FollowOutput output;
        output.index = player_index;
        output.range = ydlider_ranges[player_index];
        output_pub.publish(output);

        info.data.clear();
        info.data.push_back(calcStraight(player_point));
        info.data.push_back(0.08);
        info.data.push_back(calcAngle(player_point));
        info.data.push_back(0.5);
        move_pub.publish(info);
    }
}

void Follow::view_ydlider(const std::vector<cv::Point>& points)
{
    cv::Mat img = cv::Mat::zeros(2000, 2000, CV_8UC3);
    cv::Scalar color(0, 255, 0);
    for (auto& point : points) {
        int x = point.x+1000;
        int y = point.y+1000;
        cv::circle(img, cv::Point(x, y), 1, color, 1);
    }
    cv::circle(img, cv::Point(player_point.x+1000, player_point.y+1000), 5, cv::Scalar(0, 0, 255), 1);
    for (auto& ydlidar_point : posenet_points) {
        cv::circle(img, cv::Point((ydlidar_point.x/10)+1000, (ydlidar_point.y/10)+1000), 5,
                cv::Scalar(255, 0, 0), 1);
    }
    cv::namedWindow("window", CV_WINDOW_NORMAL);
    cv::imshow("window", img);
    cv::waitKey(1);
}

void Follow::ydlider_callback(const sensor_msgs::LaserScan::ConstPtr& msgs)
{
    /*
     * ydliderからの情報を取得
     */
    double rad = msgs->angle_min;
    ydlider_points.clear();
    ydlider_ranges.clear();
    for (int i = 0; i<(int) msgs->ranges.size(); ++i) {
        cv::Point position;
        if (msgs->range_min+0.05<msgs->ranges[i] && msgs->range_max>msgs->ranges[i]) {
            position = cv::Point(msgs->ranges[i]*sin(rad)*100, -msgs->ranges[i]*cos(rad)*100);
            ydlider_points.push_back(position);
        }
        else {
            ydlider_points.push_back(position);
        }
        ydlider_ranges.push_back(msgs->ranges[i]);
        rad += msgs->angle_increment;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "follow_me");
    Follow follow;
    while (ros::ok()) {
        ros::spinOnce();
        follow.update();
    }
    return 0;
}