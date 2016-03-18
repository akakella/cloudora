/******************************************************************
 * Abishek Akella, Cyber-Physical Cloud Computing Laboratory (2012)
 *
 * controller.cpp
 *
 * Starts node for quadrotor control. Retrieves target location 
 * data and calculates according roll and pitch PID command. Roll
 * command is based on X-axis position of the target and pitch is
 * based on the area of the object.
 *
 * Publishes target command to /ardrone/cmd_vel
 *
 *****************************************************************/

#include "controller.hpp"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <cloudora/Location.h>

controller::controller() {
	// Initialize publishers to error and drone velocity command topics
	twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	error_pub = n.advertise<std_msgs::String>("error", 1000);

	// Initialize subscriber for location data
	location_sub = n.subscribe("ardrone/target_location", 10, &controller::controllerCallBack, this);

	// Initialize currentPosition
	currentPosition.posX = 0;
	currentPosition.posY = 0;
	currentPosition.area = 0;

	/* Initialize controller parameters
	   Manually tuned for performance with AR Drone #2
	   DO NOT ADJUST
	*/
	x_max = 340;
	x_min = 300;

	area_min = 3000;
	area_max = 20000;
	area_threshold = 100;

	integ_sat_roll = 1500;
	integ_sat_pitch = 9000;
	
	Kp_roll = 0.00029;
	Ki_roll = 0.000005;
	Kd_roll = 0.00028;

	Kp_pitch = 0.00006;
	Ki_pitch = 0.0000002;
	Kd_pitch = 0.00005;
}

void controller::controllerCallBack(const cloudora::Location loc) {
	// Update current and previous positions
	cloudora::Location lastPosition = currentPosition;
	currentPosition = loc;

	double posX = currentPosition.posX;
	double lastX = lastPosition.posX;
	double area = currentPosition.area;
	double lastArea = lastPosition.area;

	// Initialize output message to zeros
	geometry_msgs::Twist twist;
	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;

	// Define sum and derivative variable for discrete-time integrator and 
	static double sum_roll = 0;
	double derivative = 0;

	static ros::Time begin_roll = ros::Time::now();

	if (posX > x_max) {
		// If target is too far to the right, calculate appropriate control command
		double deltaX = posX - ((x_max + x_min)/2);
		sum_roll = sum_roll + deltaX;

		// Integrator saturation limit
		if (sum_roll > integ_sat_roll) {
			sum_roll = integ_sat_roll;
		}

		// Calculate PID gains and set command
		derivative = posX - lastX;
		double integral = sum_roll * (ros::Time::now().toSec() - begin_roll.toSec());
		double value = -(deltaX*Kp_roll + integral*Ki_roll + derivative*Kd_roll);

		// Lower limit saturation - don't change direction
		if (value > 0) {
			value = 0;
		}
		twist.linear.y = value;
		ROS_INFO("%f", value);
	} else if (posX < x_min) {
		// If target is too far to the left, calculate appropriate control command
		double deltaX = posX - ((x_max + x_min)/2);
		sum_roll = sum_roll + deltaX;

		// Integrator saturation limit
		if (sum_roll < -integ_sat_roll) {
			sum_roll = -integ_sat_roll;
		}

		// Calculate PID gains and set command
		derivative = posX - lastX;
		double integral = sum_roll * (ros::Time::now().toSec() - begin_roll.toSec());
		double value = -(deltaX*Kp_roll + integral*Ki_roll + derivative*Kd_roll);

		// Lower limit saturation - don't change direction
		if (value < 0) {
			value = 0;
		}
		twist.linear.y = value;
		ROS_INFO("%f", value);
	} else {
		// Re-initizialize integrator to zero after settling on the target
		sum_roll = 0;
		begin_roll = ros::Time::now();
	}

	// Initialize discrete-time integrator (sum) and derivative for pitch control
	static double sum_pitch = 0;
	derivative = 0;

	static ros::Time begin_pitch = ros::Time::now();

	if (area > area_threshold && area < area_min) {
		// Move forward if target is too far away
		double delta = area - ((area_min + area_max)/2);
		sum_pitch = sum_pitch + delta;
		
		// Integrator saturation limit
		if (sum_pitch < -integ_sat_pitch) {
			sum_pitch = -integ_sat_pitch;
		}

		// Calculate and set PID command
		derivative = area - lastArea;
		double integral = sum_pitch * (ros::Time::now().toSec() - begin_pitch.toSec());
		double value = -(delta*Kp_pitch + integral*Ki_pitch + derivative*Kd_pitch);

		// Lower level limit - don't switch directions
		if (value < 0) {
			value = 0;
		}
		twist.linear.x = value;
		ROS_INFO("%f", value);
	} else if (area > area_max) {
		// Move back if object is too close
		double delta = area - ((area_min + area_max)/2);
		sum_pitch = sum_pitch + delta;

		// Integrator saturation limit
		if (sum_pitch > integ_sat_pitch) {
			sum_pitch = integ_sat_pitch;
		}

		// Calculate and set PID Command
		derivative = area - lastArea;
		double integral = sum_pitch * (ros::Time::now().toSec() - begin_pitch.toSec());
		double value = -(delta*Kp_pitch*0.25 + integral*Ki_pitch + derivative*Kd_pitch);

		// Lower level limit - don't switch directions
		if (value > 0) {
			value = 0;
		}
		twist.linear.x = value;
		ROS_INFO("%f", value);
	} else {
		// Reset if in desired window
		sum_pitch = 0;
		begin_pitch = ros::Time::now();
	}

	// If object not on screen, don't send any command
	if (area < area_threshold) {
		twist.linear.x = 0;
		twist.linear.y = 0;
	}
	twist_pub.publish(twist);
	std::stringstream s;
	s << (posX - ((x_max + x_min)/2));
	std_msgs::String msg;
	msg.data = s.str();
	error_pub.publish(msg);
}

void controller::loop() {
	ros::spin();
}

