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

namespace husky_highlevel_controller {

class HuskyHighlevelController {
public:

	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	virtual ~HuskyHighlevelController();

	void subscriberCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan);

	void publishMessage(ros::Publisher *pub_message);



private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber subscriber_;
	ros::Publisher publisher_;

	int queue_size;
	std::string topic_name;

	geometry_msgs::Twist twist_;

};

} /* name space husky_highlevel_controller */
