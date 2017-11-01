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
#include <std_srvs/SetBool.h>

namespace husky_highlevel_controller {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class HuskyHighlevelController
{
public:

	/*!
	 * Constructor.
	 * @param nodeHandle the ROS node handle.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:

	/*!
	 * Reads and verifies the ROS parameters.
	 * @return true if successful.
	 */
	bool readParameters();

	/*!
	 * ROS topic callback method.
	 * @param laserScan the received message.
	 * subscribes to topic: laserScan data and publish to a topic: cmd_vel
	 */
	void subscriberCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan);


	/*!
	 * Initialize the visualization marker topic with default parameters
	 * Elements in srv or msg definition are assigned zero values by the default
	 * no need to assign rest of the values as 0.0
	 */
	void default_Viz_marker_config();

	/*!
	 * ROS service server callback.
	 * @param request the request of the service.
	 * @param response the provided response.
	 * @return true if successful, false otherwise.
	 */

	bool serviceCallback(std_srvs::SetBool::Request& request,
			  	  	  	  std_srvs::SetBool::Response& response);

	//! ROS node handle.
	ros::NodeHandle nodeHandle_;

	//! ROS topic subscriber.
	ros::Subscriber subscriber_;

	//! ROS service server.
	ros::ServiceServer serviceServer_;

	//! ROS topic publisher - publish to /cmd_vel of husky.
	ros::Publisher publisher_;

	//! ROS topic publisher - publish to visualize marker in rViz
	ros::Publisher vis_pub;

	//! ROS messages to be publisher.
	geometry_msgs::Twist twist_;
	visualization_msgs::Marker marker_;

	//! ROS topic name to subscribe to and queue size
	std::string subscriberTopic_;
	int subscriberQueuesize_;
};

} /* name space husky_highlevel_controller */
