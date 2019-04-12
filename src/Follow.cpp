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

	ros::NodeHandle n;

	//インデックスごとの構造体を作成
	typedef struct {
		int index;
		cv::Point position;
		double existence_rate;

	} YdPoint;

	ros::Publisher pub;
	ros::Subscriber ydlider;

	std_msgs::Float64MultiArray info;

	std::vector<YdPoint> stack_data;
	std::vector<cv::Point> ydlider_msgs;

	void update();
	void minDistance_Position(const sensor_msgs::LaserScan::ConstPtr& msgs, cv::Point result);
	void closePosition(const sensor_msgs::LaserScan::ConstPtr& msgs, cv::Point base);
	double calcAngle(const sensor_msgs::LaserScan::ConstPtr& cv_image, int min_index);
	double calcStraight(const sensor_msgs::LaserScan::ConstPtr& cv_image, int mindistance);
	void ydlider_callback(const sensor_msgs::LaserScan::ConstPtr& msgs);
};

//メインクラス宣言
Follow follow;

Follow::Follow() {

	printf("Start class of 'Follow'\n");

	this->ydlider = this->n.subscribe("/scan", 1000, &Follow::ydlider_callback, this);
	this->pub = this->n.advertise<std_msgs::Float64MultiArray>("/move/amount", 1000);
}

Follow::~Follow() {
	printf("Shutdown class of 'Follow'\n");
}

void Follow::update() {

}

//ydliderからの情報を取得
void Follow::ydlider_callback(const sensor_msgs::LaserScan::ConstPtr& msgs) {
	printf("debug\n");
	//補完関数
	struct {
		// ()演算子をオーバーロードして関数オブジェクトにする
		YdPoint operator()(int first_id, int end_id) {
			return YdPoint();
		}
	} complete;

	cv::Mat img = cv::Mat::zeros(600, 600, CV_8UC3);

	double rad = msgs->angle_min;

	for (int i = 0; i < (int)msgs->ranges.size(); ++i) {

		cv::Point position = cv::Point(-1, -1);

		if (msgs->range_min + 0.05  < msgs->ranges[i] && msgs->range_max > msgs->ranges[i]) {
			//データ正常取得
			position = cv::Point(250 + msgs->ranges[i] * sin(rad) * 100, 250 + msgs->ranges[i] * cos(rad) * 100);

			cv::Scalar color(0, 0, 255);

			ydlider_msgs.push_back(position);

			cv::circle(img, position, 1, color, 1);

		} else {
			//データ取得失敗
			ydlider_msgs.push_back(position);
		}

		rad += msgs->angle_increment;

	}

	//補完実行
	for (int i = 0; i < (int)ydlider_msgs.size(); ++i) {
		if (ydlider_msgs[i].x == -1 && ydlider_msgs[i].y == -1) {
			//取得失敗
			/* code */
		}
	}


	cv::imshow("window", img);

	cv::waitKey(1);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "followme");

	while (ros::ok()) {
		ros::spinOnce();
		//follow.update();
	}

	return 0;
}