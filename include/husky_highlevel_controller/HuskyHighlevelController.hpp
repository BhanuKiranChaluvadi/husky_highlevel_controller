/*
 * HuskyHighlevelController.h
 *
 *  Created on: Oct 10, 2017
 *      Author: nth
 */

#pragma once

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

namespace husky_highlevel_controller {

class HuskyHighlevelController {
public:

	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	virtual ~HuskyHighlevelController();

	void subscriberCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan);

	void publishMessage(ros::Publisher *pub_message);

private:
	// node handle
	ros::NodeHandle nodeHandle_;

	// publishers or subscribers
	ros::Subscriber subscriber_;
	ros::Publisher publisher_;
	ros::Publisher vis_pub;

	// parameters - read from parameter file
	int queue_size;
	std::string topic_name;

	// published message
	geometry_msgs::Twist twist_;
	geometry_msgs::PointStamped closestPt_;


};

} /* name space husky_highlevel_controller */
