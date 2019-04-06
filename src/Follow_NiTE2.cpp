#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "sensor_msgs/PointCloud2.h"
#include <std_msgs/Float64MultiArray.h>

cv::Mat rgbimg;
ros::Publisher pub;
std_msgs::Float64MultiArray info;

double points[16][3];

bool flag = true;

double calcAngle(double points[]) {

	if (std::isnan(points[0]) || std::isnan(points[0]) || std::isnan(points[0])) {
		return 0;
	}

	int centerpoint = 320;

	double result;

	result = (2 * (centerpoint - points[0])) / (double)centerpoint;

	if (points[2] == 0.0) result = 0;

	return result;
}

double calcStraight(double points[]) {

	if (points[2] == -1) return -0.1;

	if (points[2] == 0) return 0;

	if (points[2] > 0.8) return (points[2]) * 0.1;

	return -0.1;
}

void exeFollow(double points[]) {

	info.data.clear();

	if (flag) {
		info.data.push_back(calcStraight(points));
	} else {
		info.data.push_back(0);
	}

	info.data.push_back(0.03);

	if (flag) {
		info.data.push_back(calcAngle(points));
	} else {
		info.data.push_back(0);
	}

	info.data.push_back(0.3);

	pub.publish(info);
}

void signal(const std_msgs::String::ConstPtr& msg) {

	if (msg->data == "start") flag = true;

	if (msg->data == "stop") flag = false;

	printf("debug\n");

}

void selectUser(const std_msgs::Float64MultiArray::ConstPtr& msg) {
	int min_index = 0;
	double min_distance = 4.0;

	if (!msg->data.empty()) {
		for (int i = 0; i < 16; ++i) {
			points[i][0] = 0.0;
			points[i][1] = 0.0;
			points[i][2] = 0.0;
		}
		for (int i = 0; i < (int)msg->data.size(); i += 4) {
			points[(int)msg->data[i] - 1][0] = msg->data[i + 1];
			points[(int)msg->data[i] - 1][1] = msg->data[i + 2];
			points[(int)msg->data[i] - 1][2] = msg->data[i + 3];
			if (msg->data[i + 3] != 0.0 && min_distance > msg->data[i + 3]) {
				min_distance = msg->data[i + 3];
				min_index = (int)msg->data[i];
			}
		}
		printf("%d\n", min_index );
		exeFollow(points[0]);
	} else {
		exeFollow(points[0]);
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "followme");

	ros::NodeHandle n;

	//ros::Subscriber sub_depth = n.subscribe("/camera/depth/image_raw", 1000, depth);
	ros::Subscriber sub = n.subscribe("/human_tracker/points", 1, selectUser);
	ros::Subscriber follow = n.subscribe("follow_me", 1000, signal);
	pub = n.advertise<std_msgs::Float64MultiArray>("/move/velocity", 1000);

	ros::spin();

	return 0;
}