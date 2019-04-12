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

	std::vector<cv::Point> ydlider_points;

	ros::NodeHandle n;

	void update();
	void view_ydlider(std::vector<cv::Point> points);
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

void Follow::view_ydlider(std::vector<cv::Point> points) {

	cv::Mat img = cv::Mat::zeros(2000, 2000, CV_8UC3);
	cv::Scalar color(0, 255, 0);
	for (int i = 0; i < (int)points.size(); ++i) {
		int x = points[i].x + 1000;
		int y = points[i].y + 1000;
		cv::circle(img, cv::Point(x, y), 1, color, 1);
	}

	cv::namedWindow("window", CV_WINDOW_NORMAL);
	cv::imshow("window", img);

	cv::waitKey(1);
}

//ydliderからの情報を取得
void Follow::ydlider_callback(const sensor_msgs::LaserScan::ConstPtr& msgs) {

	double rad = msgs->angle_min;
	ydlider_points.clear();

	for (int i = 0; i < (int)msgs->ranges.size(); ++i) {
		cv::Point position;
		if (msgs->range_min + 0.05  < msgs->ranges[i] && msgs->range_max > msgs->ranges[i]) {
			position = cv::Point(msgs->ranges[i] * sin(rad) * 100, msgs->ranges[i] * cos(rad) * 100);
			ydlider_points.push_back(position);
		} else {
			ydlider_points.push_back(position);
		}
		rad += msgs->angle_increment;
	}

	//補完
	/*for (int i = 0; i < (int)ydlider_points.size(); ++i) {
		if (ydlider_points[i].x == 0 && ydlider_points[i].y == 0) {
			//欠損
			int first_id = i - 1;
			int end_id = i + 1;

			if (i == (int)ydlider_points.size() - 1) end_id = 0;
			if (i == 0) first_id = (int)ydlider_points.size() - 1;
			int new_x = (ydlider_points[first_id].x + ydlider_points[end_id].x) / 2;
			int new_y = (ydlider_points[first_id].y + ydlider_points[end_id].y) / 2;
			cv::Point new_point = cv::Point(new_x, new_y);
			ydlider_points[i] = new_point;
		}
	}*/
	view_ydlider(ydlider_points);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "followme");

	Follow follow;

	while (ros::ok()) ros::spinOnce();

	return 0;
}