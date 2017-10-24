
This is solution for exercise4 from Programming for Robotics - ROS by ETH Zurich(http://www.rsl.ethz.ch/education-students/lectures/ros.html)

1. Dependencies: 
	1. Rviz
	2. rqt_multiplot

2. Run:
	1. Terminal 1: > roslaunch husky_highlevel_controller localization.launch
	2. In rqt_multiplot load configuration from rqtMultiplotConfig/rqt_multiplot.xml
	3. Terminal 2: 
		a. Navigate to husky_navigation.bag (downloaded from http://www.rsl.ethz.ch/education-students/lectures/ros.html)
		b. > rosbag run husky_navigation.bag --clock




