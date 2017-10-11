/*
 * HuskyHighlevelController.cpp
 *
 *  Created on: Oct 10, 2017
 *      Author: nth
 */

#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <algorithm>

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle)
{
	// TODO Auto-generated constructor stub
	subscriber_ = nodeHandle.subscribe("/scan", 10, &HuskyHighlevelController::subscriberCallback, this);

}

HuskyHighlevelController::~HuskyHighlevelController()
{
	// TODO Auto-generated destructor stub
}

void HuskyHighlevelController::subscriberCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{

//	Note that unbounded arrays in ROS msgs are mapped onto std::vector in C++.
	float smallestValue;
	std::vector<float> laserScanData = laserScan->ranges;
	smallestValue = *std::min_element(laserScanData.begin(), laserScanData.end());
	ROS_INFO(" Smallest distance measurement :[%f]", smallestValue);
}

} /* name space husky_highlevel_controller */
