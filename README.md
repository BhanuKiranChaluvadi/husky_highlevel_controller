
This is solution for exercise4 from Programming for Robotics - ROS by ETH Zurich(http://www.rsl.ethz.ch/education-students/lectures/ros.html)


1. Dependencies: 
	1. Rviz
	2. rqt_multiplot


2. Run:
	1. Update ROS bag file path in localization.launch with your local repository.
	2. Terminal : > roslaunch husky_highlevel_controller localization.launch
	3. Hit play button on rqt_mutiplot panel


3. Functionality: 
	1. Rviz is used to visualize Robot model and Point cloud generate.
	2. rqt_multiplot is used to visualize x and y postion of husky robot in the odom frame.


*********
Helpers
*********
1. In general processes launched with roslaunch have a wokring directory in $ROS_HOME (default ~/.ros).
2. In order to get input arguments eg: 	args = " -d $(find package)/../.."
					args = " --multiplot-config $(../..)"

 	rosrun rviz rviz --help
	rosrun rqt_multiplot rqt_multiplot --help
******





