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

#ifndef   FOLLOW_ME_LIB
#define   FOLLOW_ME_LIB

class FollowMeLib {
public:
	FollowMeLib();
	~FollowMeLib();
};

FollowMeLib::FollowMeLib() {
	printf("Start class of 'FollowMeLib'\n");
}

FollowMeLib::~FollowMeLib() {
	printf("Shutdown class of 'FollowMeLib'\n");
}


//インデックスごとの構造体を作成
typedef struct YdPoint {
	int index;
	double existence_rate;
} YdPoint;

#endif