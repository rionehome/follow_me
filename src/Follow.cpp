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
#include "FollowMeLib.hpp"


using namespace std;

class Follow {
public:
	Follow();
	~Follow();
	ros::Publisher pub;
	ros::Subscriber ydlider;

	std_msgs::Float64MultiArray info;

	std::vector<cv::Point> ydlider_msgs;

	ros::NodeHandle n;

	void update();
	void minDistance_Position(const sensor_msgs::LaserScan::ConstPtr& msgs, cv::Point result);
	void closePosition(const sensor_msgs::LaserScan::ConstPtr& msgs, cv::Point base);
	double calcAngle(const sensor_msgs::LaserScan::ConstPtr& cv_image, int min_index);
	double calcStraight(const sensor_msgs::LaserScan::ConstPtr& cv_image, int mindistance);
	void ydlider_callback(const sensor_msgs::LaserScan::ConstPtr& msgs);
};

Follow::Follow() {

	printf("Start class of 'Follow'\n");

	this->ydlider = n.subscribe("/scan", 1000, &Follow::ydlider_callback, this);
	this->pub = n.advertise<std_msgs::Float64MultiArray>("/move/amount", 1000);
}

Follow::~Follow() {
	printf("Shutdown class of 'Follow'\n");
}

//ydliderからの情報を取得
void Follow::ydlider_callback(const sensor_msgs::LaserScan::ConstPtr& msgs) {

	cv::Mat img = cv::Mat::zeros(600, 600, CV_8UC3);
	double rad = msgs->angle_min;

	for (int i = 0; i < (int)msgs->ranges.size(); ++i) {
		cv::Point position = cv::Point(-1, -1);
		if (msgs->range_min + 0.05  < msgs->ranges[i] && msgs->range_max > msgs->ranges[i]) {
			position = cv::Point(250 + msgs->ranges[i] * sin(rad) * 100, 250 + msgs->ranges[i] * cos(rad) * 100);
			cv::Scalar color(0, 255, 0);
			cv::circle(img, position, 1, color, 1);
			ydlider_msgs.push_back(position);
		} else {
			ydlider_msgs.push_back(position);
		}
		rad += msgs->angle_increment;
	}

	cv::imshow("window", img);
	cv::waitKey(1);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "followme");

	Follow follow;

	while (ros::ok()) ros::spinOnce();

	return 0;
}