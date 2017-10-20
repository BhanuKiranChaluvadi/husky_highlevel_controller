/*
 * marker_listener.cpp
 *
 *  Created on: Oct 16, 2017
 *      Author: robotics
 */

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <visualization_msgs/Marker.h>

namespace husky_highlevel_controller {

class MarkerPublish
{
public:
	MarkerPublish(ros::NodeHandle& nodeHandle): node_(nodeHandle), tf2Listener_(tf2Buffer_), target_frame_("odom"), tf2_filter_(point_sub_, tf2Buffer_, target_frame_, 10, 0)
	{
		ROS_INFO_STREAM("XXXXXXXXXXXXX Entered constructor XXXXXXXXXXXXXXXXX");
		// publisher
		marker_Visualization_pub = node_.advertise<visualization_msgs::Marker>("/vizMarker", 10);
		// published message configuration
		marker_msg.ns = "/";
		marker_msg.type = visualization_msgs::Marker::SPHERE;
		marker_msg.pose.position.z = 0;
		marker_msg.color.a = 1.0;
		marker_msg.color.g = 1.0;
		// subscribing
		point_sub_.subscribe(node_, "/closestPoint", 10);
		// subscriber callback
		tf2_filter_.registerCallback(boost::bind(&MarkerPublish::pointStampedCallback, this, _1));
	}

	// callback to register with tf2_ros::MessageFilter to be called when transforms are available
	void pointStampedCallback(const geometry_msgs::PointStamped::ConstPtr& laserBasePt_ptr)
	{
		geometry_msgs::PointStamped OdomBasePt;

		try {
			ROS_DEBUG("Hello %s", "callback - entered");
			tf2Buffer_.transform(*laserBasePt_ptr, OdomBasePt, target_frame_);

			// marker
			marker_msg.header = laserBasePt_ptr->header;
			marker_msg.pose.position.x = laserBasePt_ptr->point.x;
			marker_msg.pose.position.y = laserBasePt_ptr->point.y;

			// publish
			marker_Visualization_pub.publish(marker_msg);
			ROS_DEBUG("Hello %s", "callback - exist");
		}
		catch(tf2::TransformException &ex){
			ROS_DEBUG("Hello %s", "catch - entered");
			ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
		}
	}

private:

	ros::NodeHandle node_;

	std::string target_frame_;
	tf2_ros::TransformListener tf2Listener_;
	tf2_ros::Buffer tf2Buffer_;

	message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
	tf2_ros::MessageFilter<geometry_msgs::PointStamped> tf2_filter_;

	ros::Publisher marker_Visualization_pub;
	visualization_msgs::Marker marker_msg;
};

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vizMarker");
	ros::NodeHandle node("~");
	husky_highlevel_controller::MarkerPublish marker_publish(node);
	ros::spin();
	return 0;
};


