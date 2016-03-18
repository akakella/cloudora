/******************************************************************
 * Abishek Akella, Cyber-Physical Cloud Computing Laboratory (2012)
 *
 * droneController.cpp
 *
 * Starts node for feedback tracking control
 *
 *****************************************************************/

#include "controller.hpp"
#include "ros/ros.h"

int main(int argc, char**argv)
{
	ros::init(argc, argv, "object_tracking");

	ROS_INFO("Started Object Tracking Node");

	controller control;
	control.loop();

	return 0;
}
