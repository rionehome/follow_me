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


using namespace std;

vector<cv::Point> stack_point;
cv::Point base_point;


ros::Publisher pub;
std_msgs::Float64MultiArray info;

void minDistance_Position(const sensor_msgs::LaserScan::ConstPtr& msgs, cv::Point result) {

	double min = INT_MAX;
	double min_index = 0;
	double rad = msgs->angle_min;

	for (int i = 0; i < (int)msgs->ranges.size(); ++i) {
		if (msgs->range_min + 0.05 < msgs->ranges[i] && msgs->range_max > msgs->ranges[i]) {
			if (min > msgs->ranges[i]) {
				min = msgs->ranges[i];
				min_index = i;
			}
		}
		rad += msgs->angle_increment;
	}
	result.x = 250 + msgs->ranges[min_index] * sin(rad) * 100;
	result.y = 250 + msgs->ranges[min_index] * cos(rad) * 100;
	//stack_point.push_back(cv::Point2f(250 + msgs->ranges[min_index] * sin(rad) * 100, 250 + msgs->ranges[min_index] * cos(rad) * 100));
	//data[0] = min;
	//data[1] = min_index;

}

void closePosition(const sensor_msgs::LaserScan::ConstPtr& msgs, cv::Point base) {
	double rad = msgs->angle_min;
	cv::Point position, min(0, 0);
	double min_distance = msgs->range_max;
	double distance;
	for (int i = 0; i < (int)msgs->ranges.size(); ++i) {
		if (msgs->range_min + 0.05  < msgs->ranges[i] && msgs->range_max > msgs->ranges[i]) {
			position.x = 250 + msgs->ranges[i] * sin(rad) * 100;
			position.y = 250 + msgs->ranges[i] * cos(rad) * 100;
			distance = hypot(position.x - base.x, position.y - base.y);
			if (min_distance > distance) {
				min_distance = distance;
				min.x = position.x;
				min.y = position.y;
			}
		}
		rad += msgs->angle_increment;
	}
	base.x = min.x;
	base.y = min.y;
}

double calcAngle(const sensor_msgs::LaserScan::ConstPtr& cv_image, int min_index) {

	int centerindex = 562;

	double result;

	result = (5 * (min_index - centerindex)) / 518.0;
	return result;
}

double calcStraight(const sensor_msgs::LaserScan::ConstPtr& cv_image, int mindistance) {
	return mindistance > 0 ? 0.2 : 0;
}

void depth(const sensor_msgs::LaserScan::ConstPtr& msgs) {

	cv::Mat img = cv::Mat::zeros(600, 600, CV_8UC3);

	double rad = msgs->angle_min;
	double data[2] = {0, 0};

	cout << base_point << '\n';
	/*
		if (stack_point.empty()) {
			//最初のポイントを決定
			minDistance_Position(msgs, stack_point[0]);
		} else {
			closePosition(msgs, stack_point[0]);
		}*/
	info.data.clear();

	info.data.push_back(calcStraight(msgs, data[0]));
	//info.data.push_back(0);
	info.data.push_back(calcAngle(msgs, data[1]));

	pub.publish(info);

	for (int i = 0; i < (int)msgs->ranges.size(); ++i) {

		if (msgs->range_min + 0.05  < msgs->ranges[i] && msgs->range_max > msgs->ranges[i]) {

			cv::Point position(250 + msgs->ranges[i] * sin(rad) * 100, 250 + msgs->ranges[i] * cos(rad) * 100);
			//cv::Point position(50, 50);
			cv::Scalar color;
			if (i == 0) {
				color = cv::Scalar(0, 255, 0);

				printf("red\n");
			} else {
				color = cv::Scalar(0, 0, 255);

			}

			//cv::drawMarker(img, position, color);

			cv::circle(img, position, 1, color, 1);

			//printf("[ %f, %f ]\n", msgs->ranges[i] * sin(rad) * 100, msgs->ranges[i] * cos(rad) * 100);
		}

		rad += msgs->angle_increment;

	}

	cv::imshow("window", img);

	cv::waitKey(1);

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "followme_ydlider");

	ros::NodeHandle n;

	ros::Subscriber sub_depth = n.subscribe("/scan", 1000, depth);
	pub = n.advertise<std_msgs::Float64MultiArray>("/move/amount", 1000);


	ros::spin();

	return 0;
}