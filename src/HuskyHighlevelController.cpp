/*
 * HuskyHighlevelController.cpp
 *
 *  Created on: Oct 10, 2017
 *      Author: nth
 */

#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <algorithm>
#include <math.h>

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle)
{
	// TODO Auto-generated constructor stub
	if (!nodeHandle_.getParam("/husky_highlevel_controller/scan_subscriber_queue_size", queue_size))
		ROS_ERROR("Failed to get param 'scan_subscriber_queue_size'");

	if (!nodeHandle_.getParam("/husky_highlevel_controller/scan_subscriber_topic_name", topic_name))
		ROS_ERROR("Failed to get param 'scan_subscriber_topic_name'");

	twist_.linear.x = 0.6;
	twist_.linear.y = 0.0;
	twist_.linear.z = 0.0;

	twist_.angular.x = 0.0;
	twist_.angular.y = 0.0;
	twist_.angular.z = 0.0;


	subscriber_ = nodeHandle.subscribe(topic_name, queue_size, &HuskyHighlevelController::subscriberCallback, this);
	publisher_ = nodeHandle.advertise <geometry_msgs::Twist> ("/cmd_vel", 10);
}

HuskyHighlevelController::~HuskyHighlevelController()
{
	// TODO Auto-generated destructor stub
}

void HuskyHighlevelController::subscriberCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{

//	Note that unbounded arrays in ROS msgs are mapped onto std::vector in C++.
	int minRayIndex;
	float pGain = 0.2;
	std::vector<float> laserScanData = laserScan->ranges;
	minRayIndex = std::distance(laserScanData.begin(), std::min_element(laserScanData.begin(), laserScanData.end()));
	// with respect to x-axis. And x-axis is pointed in forward direction.
	float minRayAngle = laserScan->angle_min + minRayIndex * laserScan->angle_increment;

	twist_.angular.z = -pGain*sin(minRayAngle);
	publishMessage(&publisher_);

}

void HuskyHighlevelController::publishMessage(ros::Publisher *pub_message)
{
	pub_message->publish(twist_);
}

} /* name space husky_highlevel_controller */
