#include <husky_highlevel_controller/Controller.h>

namespace husky_highlevel_controller {

Controller::Controller()
	: minRayIndex(0), Obstacle_distance(100.0), minRayAngle(0.0)
{
	default_Viz_marker_config();
}

Controller::~Controller()
{
	// TODO Auto-generated destructor stub
}

void Controller::findMinrayAngle(const sensor_msgs::LaserScan& laserScan)
{
	std::vector<float> laserScanData = laserScan.ranges;

	// laserScan ranges minimum distance details
	minRayIndex = std::distance(laserScanData.begin(), std::min_element(laserScanData.begin(), laserScanData.end()));

	Obstacle_distance = laserScan.ranges[minRayIndex];

	// with respect to x-axis. And x-axis is pointed in forward direction.
	minRayAngle = laserScan.angle_min + minRayIndex * laserScan.angle_increment;
}


void Controller::findTwist()
{
	twist_.linear.x = 3.0;
	twist_.angular.z = -pGain*sin(minRayAngle);
}

void Controller::getMarker()
{
	marker_.header.stamp = ros::Time();
	marker_.pose.position.x = Obstacle_distance * cos(minRayAngle);
	marker_.pose.position.y = -Obstacle_distance * sin(minRayAngle);
}

void Controller::default_Viz_marker_config()
{
	marker_.header.frame_id = "base_laser";
	marker_.header.stamp = ros::Time();
	marker_.ns = "/husky_highlevel_controller";
	marker_.id = 0;
	marker_.type = visualization_msgs::Marker::SPHERE;
	marker_.action = visualization_msgs::Marker::ADD;
	marker_.pose.position.x = 0.0;
	marker_.pose.position.y = 0.0;
	marker_.pose.position.z = 0.0;
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

} /* namespace husky_highlevel_controller */
