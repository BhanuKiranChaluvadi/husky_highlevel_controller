/*
 * HuskyHighlevelController.h
 *
 *  Created on: Oct 10, 2017
 *      Author: nth
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace husky_highlevel_controller {

class HuskyHighlevelController {
public:

	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	virtual ~HuskyHighlevelController();

	void subscriberCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan);

	void subscribe();


private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber subscriber_;
};

} /* name space husky_highlevel_controller */
