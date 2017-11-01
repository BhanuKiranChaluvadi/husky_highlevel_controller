/*
 * HuskyHighlevelController.cpp
 *
 *  Created on: Oct 10, 2017
 *      Author: nth
 */

#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

// STD
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
	twist_.linear.x = 3.0;
	default_Viz_marker_config();

	// initialize publishers and subscribers
	subscriber_ = nodeHandle_.subscribe(subscriberTopic_, subscriberQueuesize_,
											&HuskyHighlevelController::subscriberCallback, this);

	serviceServer_ = nodeHandle_.advertiseService("run",
	                                                &HuskyHighlevelController::serviceCallback, this);

	publisher_ = nodeHandle_.advertise <geometry_msgs::Twist> ("/cmd_vel", 10);

	vis_pub = nodeHandle_.advertise <visualization_msgs::Marker> ("visualization_marker", 0 );

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

	// publish- marker
	marker_.pose.position.x = distance * cos(minRayAngle);
	marker_.pose.position.y = -distance * sin(minRayAngle);
	vis_pub.publish(marker_);

	// publish - cmd_vel
	twist_.angular.z = -pGain*sin(minRayAngle);
	publisher_.publish(twist_);


}

bool HuskyHighlevelController::serviceCallback(std_srvs::SetBool::Request& request,
  	  	  	  	  	  	  	  	  	  	  	  	  std_srvs::SetBool::Response& response)
{
	response.success = true;
	response.message = "The process is running: " + std::to_string(request.data);
	return true;
}

void HuskyHighlevelController::default_Viz_marker_config()
{
	marker_.header.frame_id = "base_laser";
	marker_.header.stamp = ros::Time();
	marker_.ns = "/husky_highlevel_controller";
	marker_.id = 0;
	marker_.type = visualization_msgs::Marker::SPHERE;
	marker_.action = visualization_msgs::Marker::ADD;
	marker_.pose.position.x = 1;
	marker_.pose.position.y = 1;
	marker_.pose.position.z = 1;
	marker_.pose.orientation.x = 0.0;
	marker_.pose.orientation.y = 0.0;
	marker_.pose.orientation.z = 0.0;
	marker_.pose.orientation.w = 1.0;
	marker_.scale.x = 1;
	marker_.scale.y = 1;
	marker_.scale.z = 1;
	marker_.color.a = 1.0; // Don't forget to set the alpha!
	marker_.color.r = 0.0;
	marker_.color.g = 1.0;
	marker_.color.b = 0.0;
	//only if using a MESH_RESOURCE marker type:
	marker_.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
}

} /* name space husky_highlevel_controller */
