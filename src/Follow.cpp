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
//#include "ros_posenet/Keypoint.h"
//#include "ros_posenet/Poses.h"
//#include "ros_posenet/Pose.h"


using namespace std;

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
	ros::Subscriber ydlider;
	//ros::Subscriber posenet;
	ros::Subscriber signal;


	std_msgs::Float64MultiArray info;
	std::vector<cv::Point> ydlider_points;
	std::vector<cv::Point> posenet_points;
	std::vector<SampleData> data_list;

	int player_index{};
	bool status = true;
	cv::Point player_point;

	ros::NodeHandle n;

	void update();

	static double calc_normal_distribution(int target_index, int center_index, int index_size);

	static double cost(const cv::Point &p1, const cv::Point &p2) {
		double result = p2.x == 0 && p2.y == 0 ? 0.01 : 3 / hypot(p2.x - p1.x, p2.y - p1.y);
		if (isinf(result)) return 0.1;
		return result;
	}

	void view_ydlider(const std::vector<cv::Point> &points);

	static double calcAngle(const cv::Point &target_point);

	static double calcStraight(const cv::Point &target_point);

	void signal_callback(const std_msgs::String::ConstPtr &msgs) {
		status = msgs->data == "start";
	}

	void ydlider_callback(const sensor_msgs::LaserScan::ConstPtr &msgs);

	//void posenet_callback(const ros_posenet::Poses::ConstPtr &msg);
};

Follow::Follow() {

	printf("Start class of 'Follow'\n");

	this->ydlider = n.subscribe("/scan", 1, &Follow::ydlider_callback, this);
	//this->posenet = n.subscribe("/ros_posenet/poses", 1, &Follow::posenet_callback, this);
	this->move_pub = n.advertise<std_msgs::Float64MultiArray>("/move/velocity", 1000);
	this->signal = n.subscribe("/follow_me_nlp/follow_me", 1, &Follow::signal_callback, this);
}

Follow::~Follow() {
	printf("Shutdown class of 'Follow'\n");
}

double Follow::calc_normal_distribution(int target_index, int center_index, int index_size) {
	double index_distance = abs(target_index - center_index);
	if (index_distance > index_size / 2) index_distance -= index_size;
	index_distance *= 1 / 80.0;
	double normal_distribution = (1 / sqrt(2.0 * M_PI)) * exp((-index_distance * index_distance) / 2.0);
	return normal_distribution;
}

double Follow::calcAngle(const cv::Point &target_point) {

	double result = -target_point.x * 0.015;
	printf("a %f\n", result);

	/*
	int centerindex = (int)ydlider_points.size() / 2;

	double result;

	result = (5 * (min_index - centerindex)) / 518.0;
	*/
	return result;
}

double Follow::calcStraight(const cv::Point &target_point) {
	double result = abs(target_point.y) > 60 ? target_point.y * 0.002 : 0;
	printf("s %f\n", result);
	if (result > 0.4) result = 0.4;
	return result;
}

void Follow::update() {
	if ((int) ydlider_points.size() == 0) return;

	if (data_list.empty()) {
		//初期化
		for (int i = 0; i < (int) ydlider_points.size(); ++i) {
			//正規分布に従って初期化
			//double pole_index = i / 10.0;
			//if (i > (int)ydlider_points.size() / 2.0) pole_index = (i - (int)ydlider_points.size()) / 10.0;
			//double normal_distribution = (1 / sqrt(2.0 * M_PI)) * exp((-pole_index * pole_index) / 2.0);
			SampleData d = {i, ydlider_points[i], calc_normal_distribution(i, 0, (int) ydlider_points.size()) *
			                cost(cv::Point(0, 0), ydlider_points[i])
			               };
			data_list.push_back(d);
		}
	} else {
		//更新
		for (int i = 0; i < (int) ydlider_points.size(); ++i) {
			data_list[i].point = ydlider_points[i];
			//data_list[i].existence_rate = data_list[i].existence_rate + cost(ydlider_points[i]) + (normal_distribution / 10);
			data_list[i].existence_rate = cost(player_point, ydlider_points[i]) + data_list[i].existence_rate *
			                              calc_normal_distribution(i,
			                                      player_index,
			                                      (int) ydlider_points.size());
			//data_list[i].existence_rate * calc_normal_distribution(i, 0, (int)ydlider_points.size()) * 2;
			//printf("%f\n", data_list[i].existence_rate );
			//data_list[i].existence_rate * calc_normal_distribution(i, player_index, (int)ydlider_points.size());
			//data_list[i].existence_rate * calc_normal_distribution(i, 0, (int)ydlider_points.size()) * 2;
		}
	}
	//正規化
	double total = 0.0;
	double max = 0.0;
	int max_index = 0;

	for (int i = 0; i < (int) ydlider_points.size(); ++i) {
		if (data_list[i].existence_rate == 0) data_list[i].existence_rate = 0.001;
		total += data_list[i].existence_rate;
		if (max < data_list[i].existence_rate) {
			max = data_list[i].existence_rate;
			max_index = i;
		}
	}
	for (int i = 0; i < (int) ydlider_points.size(); ++i) {
		data_list[i].existence_rate *= 1 / total;
	}

	player_index = max_index;
	player_point = cv::Point(ydlider_points[player_index]);

	//cout << player_point << '\n';
	//printf("%d\n", player_index );
	//printf("%f\n", data_list[max_index].existence_rate );

	//制御
	info.data.clear();

	info.data.push_back(calcStraight(player_point));

	info.data.push_back(0.03);

	info.data.push_back(calcAngle(player_point));

	info.data.push_back(0.5);

	//シグナルがtrueの時のみ実行
	if (status)
		move_pub.publish(info);

}

void Follow::view_ydlider(const std::vector<cv::Point> &points) {

	cv::Mat img = cv::Mat::zeros(2000, 2000, CV_8UC3);
	cv::Scalar color(0, 255, 0);
	for (auto &point : points) {
		int x = point.x + 1000;
		int y = point.y + 1000;
		cv::circle(img, cv::Point(x, y), 1, color, 1);
	}

	cv::circle(img, cv::Point(player_point.x + 1000, player_point.y + 1000), 5, cv::Scalar(0, 0, 255), 1);
	for (auto &posenet_point : posenet_points) {
		cv::circle(img, cv::Point((posenet_point.x / 10) + 1000, (posenet_point.y / 10) + 1000), 5,
		           cv::Scalar(255, 0, 0), 1);
	}
	cv::namedWindow("window", CV_WINDOW_NORMAL);
	cv::imshow("window", img);

	cv::waitKey(1);
}

//ydliderからの情報を取得
void Follow::ydlider_callback(const sensor_msgs::LaserScan::ConstPtr &msgs) {

	double rad = msgs->angle_min;
	ydlider_points.clear();

	for (int i = 0; i < (int) msgs->ranges.size(); ++i) {
		cv::Point position;
		if (msgs->range_min + 0.05 < msgs->ranges[i] && msgs->range_max > msgs->ranges[i]) {
			position = cv::Point(msgs->ranges[i] * sin(rad) * 100, -msgs->ranges[i] * cos(rad) * 100);
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

//posenet
/*
void Follow::posenet_callback(const ros_posenet::Poses::ConstPtr &msg) {
    //posenetの情報を取得、X座標のみ保持
    posenet_points.clear();
    for (const auto &pose : msg->poses) {
        for (const auto &keypoint : pose.keypoints) {
            if (keypoint.position.z != 0)
                posenet_points.emplace_back(keypoint.position.z, keypoint.position.x);
        }
    }
}
*/

int main(int argc, char **argv) {

	ros::init(argc, argv, "followme");

	Follow follow;

	while (ros::ok()) {
		ros::spinOnce();
		follow.update();
	}

	return 0;
}