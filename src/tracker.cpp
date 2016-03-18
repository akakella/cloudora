/******************************************************************
 * Abishek Akella, Cyber-Physical Cloud Computing Laboratory (2012)
 *
 * tracker.cpp
 *
 * Starts node for object tracking
 *
 *****************************************************************/

#include "targetLocation.hpp"
#include "ros/ros.h"

int main(int argc, char**argv)
{
	ros::init(argc, argv, "object_tracking");

	ROS_INFO("Started Object Tracking Node");

	targetLocation locator;
	locator.loop();

	return 0;
}
