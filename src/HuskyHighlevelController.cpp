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
	if (!readParameters()) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}

	// Elements in srv or msg definition are assigned zero values by the default
	// no need to assign rest of the values as 0.0
	twist_.linear.x = 2.0;

	// initialize publishers and subscribers
	subscriber_ = nodeHandle_.subscribe(subscriberTopic_, subscriberQueuesize_,
												&HuskyHighlevelController::subscriberCallback, this);

	publisher_ = nodeHandle_.advertise <geometry_msgs::Twist> ("/cmd_vel", 10);

	vis_pub = nodeHandle.advertise <geometry_msgs::PointStamped> ("closestPoint", 0 );

	ROS_INFO("Successfully launched node.");

}

HuskyHighlevelController::~HuskyHighlevelController()
{
	// TODO Auto-generated destructor stub
}

bool HuskyHighlevelController::readParameters()
{
	if (!nodeHandle_.getParam("subscriber_queue_size", subscriberQueuesize_) ||
			!nodeHandle_.getParam("subscriber_topic", subscriberTopic_))  return false;

	return true;
}

void HuskyHighlevelController::subscriberCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{

//	Note that unbounded arrays in ROS msgs are mapped onto std::vector in C++.
	int minRayIndex;
	float pGain = 0.5;
	std::vector<float> laserScanData = laserScan->ranges;

	// laserScan ranges minimum distance details
	minRayIndex = std::distance(laserScanData.begin(), std::min_element(laserScanData.begin(), laserScanData.end()));
	float distance = laserScan->ranges[minRayIndex];
	// with respect to x-axis. And x-axis is pointed in forward direction.
	float minRayAngle = laserScan->angle_min + minRayIndex * laserScan->angle_increment;

	// publish- marker position
	closestPt_.header = laserScan->header;
	closestPt_.point.x = distance * cos(minRayAngle);
	closestPt_.point.y = -distance * sin(minRayAngle);
	vis_pub.publish(closestPt_);

	// publish - cmd_vel
	twist_.angular.z = -pGain*sin(minRayAngle);
	publisher_.publish(twist_);

}

} /* name space husky_highlevel_controller */
