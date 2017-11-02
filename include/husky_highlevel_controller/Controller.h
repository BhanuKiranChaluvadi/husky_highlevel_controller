
#pragma once

#include <vector>
#include <math.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

namespace husky_highlevel_controller {

/*!
 * Class containing the algorithmic part of the package.
 */

class Controller
{
public:
	/*!
	 * Constructor.
	 */
	Controller();

	/*!
	 * Destructor.
	 */
	virtual ~Controller();

	/*!
	 * Finds minimum distance recorded from ranges
	 * and find the angle associated with this distance
	 * @param laserScan obtained from laser scan.
	 * Assign values to all private variables
	 */
	void findMinrayAngle(const sensor_msgs::LaserScan& laserScan);

	/*!
	 * Controller - assign values to twist_ variable
	 */
	void findTwist();

	void default_Viz_marker_config();
	void getMarker();

private:


	int minRayIndex;

	float Obstacle_distance;

	float minRayAngle;

	float pGain = 0.5;

public:

	geometry_msgs::Twist twist_;
	visualization_msgs::Marker marker_;

};

} /* namespace husky_highlevel_controller */

