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

using namespace std;

//URG maxサイズ:1080
//URG minサイズ:44

ros::Publisher pub;
std_msgs::Float64MultiArray info;

void minDistance(const sensor_msgs::LaserScan::ConstPtr& msgs, double data[]) {

	double min = INT_MAX;
	double min_index = 0;

	for (int i = 44; i < (int)msgs->ranges.size(); ++i) {

		if (msgs->range_min + 0.5 < msgs->ranges[i] && msgs->range_max > msgs->ranges[i]) {

			if (min > msgs->ranges[i]) {
				min = msgs->ranges[i];
				min_index = i;
			}

		}

	}

	data[0] = min;
	data[1] = min_index;

}

double calcAngle(const sensor_msgs::LaserScan::ConstPtr& cv_image, int min_index) {

	int centerindex = 562;

	double result;

	result = (5 * (min_index - centerindex)) / 518.0;
	printf("%f\n", result );
	return result;
}

double calcStraight(const sensor_msgs::LaserScan::ConstPtr& cv_image, int mindistance) {

	return mindistance > 0 ? 0.2 : 0;

}

void depth(const sensor_msgs::LaserScan::ConstPtr& msgs) {

	cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);

	double rad = msgs->angle_min;
	double data[2] = {0, 0};


	minDistance(msgs, data);

	info.data.clear();

	info.data.push_back(calcStraight(msgs, data[0]));
	//info.data.push_back(0);
	info.data.push_back(calcAngle(msgs, data[1]));

	pub.publish(info);

	for (int i = 44; i < (int)msgs->ranges.size(); ++i) {

		if (msgs->range_min + 0.7 < msgs->ranges[i] && msgs->range_max > msgs->ranges[i]) {

			cv::Point position(250 + msgs->ranges[i] * sin(rad) * 100, 250 + msgs->ranges[i] * cos(rad) * 100);
			//cv::Point position(50, 50);
			cv::Scalar color(0, 255, 0);

			//cv::drawMarker(img, position, color);

			cv::circle(img, position, 1, color, -1);

			//printf("[ %f, %f ]\n", msgs->ranges[i] * sin(rad) * 100, msgs->ranges[i] * cos(rad) * 100);
		}

		rad += msgs->angle_increment;

	}

	cv::imshow("window", img);

	cv::waitKey(1);

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "followme_urg");

	ros::NodeHandle n;

	ros::Subscriber sub_depth = n.subscribe("/scan", 1000, depth);
	pub = n.advertise<std_msgs::Float64MultiArray>("/move/amount", 1000);


	ros::spin();

	return 0;
}