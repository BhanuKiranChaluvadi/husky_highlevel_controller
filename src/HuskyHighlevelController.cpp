/*
 * HuskyHighlevelController.cpp
 *
 *  Created on: Oct 10, 2017
 *      Author: nth
 */

#include "husky_highlevel_controller/HuskyHighlevelController.hpp"


namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle)
{
	if (!readParameters()) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}

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

void HuskyHighlevelController::subscriberCallback(const sensor_msgs::LaserScan& laserScan)
{
	controller_.findMinrayAngle(laserScan);
	controller_.findTwist();
	controller_.getMarker();

	if(status_){
		publisher_.publish(controller_.twist_);
		vis_pub.publish(controller_.marker_);
	}
}

bool HuskyHighlevelController::serviceCallback(std_srvs::SetBool::Request& request,
  	  	  	  	  	  	  	  	  	  	  	  	  std_srvs::SetBool::Response& response)
{
	status_ = request.data;
	response.success = true;
	response.message = "The process is running: " + std::to_string(request.data);
	return true;
}

} /* name space husky_highlevel_controller */
