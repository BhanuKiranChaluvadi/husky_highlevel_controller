/*
 * HuskyHighlevelController.h
 *
 *  Created on: Oct 10, 2017
 *      Author: nth
 */

#pragma once

#include <ros/ros.h>

namespace husky_highlevel_controller {

class HuskyHighlevelController {
public:
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);
	virtual ~HuskyHighlevelController();

private:
	ros::NodeHandle nodeHandle_;
};

} /* name space husky_highlevel_controller */
