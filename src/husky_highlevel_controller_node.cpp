/*
 * husky_highlevel_controller_node.cpp
 *
 *  Created on: Oct 10, 2017
 *      Author: nth
 */


#include <ros/ros.h>
#include <husky_highlevel_controller/HuskyHighlevelController.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "husky_highlevel_controller");

	ros::NodeHandle nodeHandle("~");

	husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nodeHandle);

	ros::spin();

	return 0;
}

