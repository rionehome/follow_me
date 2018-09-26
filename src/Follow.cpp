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

bool flag = true;

/*double calcAngle(const sensor_msgs::Image::ConstPtr& cv_image, int minpoint[]) {

	int centerpoint = cv_image->width / 2;

	double result;

	result = (2 * (centerpoint - minpoint[1])) / (double)centerpoint;

	return result;
}*/


double calcAngle(cv::Mat map, cv::Point_<int> point, double min_distance) {

	int centerpoint = map.size[1] / 2;

	double result;

	result = (2 * (centerpoint - point.x)) / (double)centerpoint;

	if (min_distance == -1) result = 0;

	return result;
}

double calcStraight(double min_distance) {

	if (min_distance == -1) return -0.1;

	if (min_distance == 0) return 0;

	if (min_distance > 0.5) return (min_distance) * 0.3;

	return -0.1;
}

/*void minDistancePoint(const sensor_msgs::Image::ConstPtr& cv_image , int point[]) {

	int height = cv_image->height;
	int width = cv_image->width;
	int min = INT_MAX;
	int count = 0;
	int sum = 0;

	for (int i = (height / 3) * width + 1; i < (height / 3) * width + width; ++i) {

		if (min > cv_image->data[i] && cv_image->data[i] > 0) min = cv_image->data[i];

	}

	for (int i = (height / 3) * width + 1; i < (height / 3) * width + width; ++i) {

		if (min == cv_image->data[i]) {
			count++;
			sum += i;

			cv::Point position(i - ((height / 3) * width), height / 3);

			cv::Scalar color(0, 255, 0);

			cv::drawMarker(rgbimg, position, color);
		}

	}

	if (count == 0) return;

	point[0] = height / 3;
	point[1] = (sum / count) - ((height / 3) * width);
	point[2] = min;
	//printf("%d , %d\n", point[0], point[1] );
	//printf("%d\n", min );

}*/

cv::Point_<int> minDistancePoint(cv::Mat map, double *min) {

	double min_distance = std::numeric_limits<double>::max();
	double sample;
	cv::Point_<int> result;

	int y = map.size[0] / 2;
	int count = 0;

	for (int x = 0; x < map.size[1]; ++x) {

		sample = map.at<double>(y, x);

		if (sample > 0 && min_distance > sample) {
			min_distance = sample;
			result.x = x;
			result.y = y;
		}

		if (std::isnan(sample)) count++;

	}

	printf("%d\n", count );

	*min = (map.size[1] / 2 > count ? min_distance : -1);

	return result;
}

cv::Mat calcDepth(const sensor_msgs::PointCloud2::ConstPtr& msg) {

	int height = msg->height;
	int width = msg->width;

	cv::Mat result(height, width, CV_MAKETYPE(CV_64F, 1));

	for (int v = 0; v < height; ++v) {
		for (int u = 0; u < width; ++u) {

			// Convert from u (column / width), v (row/height) to position in array
			// where X,Y,Z data starts
			int arrayPosition = v * msg->row_step + u * msg->point_step;

			// compute position in array where x,y,z data start
			int arrayPosX = arrayPosition + msg->fields[0].offset; // X has an offset of 0
			int arrayPosY = arrayPosition + msg->fields[1].offset; // Y has an offset of 4
			int arrayPosZ = arrayPosition + msg->fields[2].offset; // Z has an offset of 8

			float X = 0.0;
			float Y = 0.0;
			float Z = 0.0;

			memcpy(&X, &msg->data[arrayPosX], sizeof(float));
			memcpy(&Y, &msg->data[arrayPosY], sizeof(float));
			memcpy(&Z, &msg->data[arrayPosZ], sizeof(float));

			result.at<double>(v, u) = sqrt(pow(X, 2) + pow(Y, 2) + pow(Z, 2));

		}
	}

	return result;

}

void depth_points(const sensor_msgs::PointCloud2::ConstPtr& msg ) {

	cv::Mat depth_image = calcDepth(msg);

	cv::Point_<int> position;
	double min_distance;

	position = minDistancePoint(depth_image, &min_distance);

	cv::Scalar color(0, 0, 255);

	cv::drawMarker(rgbimg, position, color);

	info.data.clear();

	if (flag) {
		info.data.push_back(calcStraight(min_distance));
	} else {
		info.data.push_back(0);
	}

	info.data.push_back(0.03);

	if (flag) {
		info.data.push_back(calcAngle(depth_image, position, min_distance));
	} else {
		info.data.push_back(0);
	}

	info.data.push_back(0.3);

	pub.publish(info);

	cv::imshow("image", rgbimg);

	depth_image.convertTo(depth_image, CV_8UC1, 255 / 5 , 0);

	cv::imshow("window", depth_image);

	cv::waitKey(1);

}

/*void depth(const sensor_msgs::Image::ConstPtr& msg) {

	int point[3] = {0, 0, 0};
	cv::Mat mat;

	minDistancePoint(msg, point);

	cv::Point position(point[1], point[0]);

	cv::Scalar color(0, 0, 255);

	cv::drawMarker(rgbimg, position, color);

	info.data.clear();

	if (flag) {
		info.data.push_back(calcStraight(msg, point[2]));
	} else {
		info.data.push_back(0);
	}

	info.data.push_back(0.03);

	if (flag) {
		info.data.push_back(calcAngle(msg, point));
	} else {
		info.data.push_back(0);
	}

	info.data.push_back(0.3);

	pub.publish(info);

	cv::imshow("image", rgbimg);
	cv::waitKey(1);

}
*/


void rgb(const sensor_msgs::Image::ConstPtr& msg) {

	cv_bridge::CvImagePtr cv_rgb;

	try {
		// ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
		cv_rgb = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	rgbimg = cv_rgb->image;

}

void signal(const std_msgs::String::ConstPtr& msg) {

	if (msg->data == "start") flag = true;

	if (msg->data == "stop") flag = false;

	printf("debug\n");

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "followme");

	ros::NodeHandle n;

	//ros::Subscriber sub_depth = n.subscribe("/camera/depth/image_raw", 1000, depth);
	ros::Subscriber sub = n.subscribe("/camera/depth/points", 1, depth_points);
	ros::Subscriber follow = n.subscribe("follow_me", 1000, signal);
	ros::Subscriber followsignal = n.subscribe("/camera/rgb/image_raw", 1, rgb);
	pub = n.advertise<std_msgs::Float64MultiArray>("/move/velocity", 1000);

	ros::spin();

	return 0;
}