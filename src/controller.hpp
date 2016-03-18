/******************************************************************
 * Abishek Akella, Cyber-Physical Cloud Computing Laboratory (2012)
 *
 * controller.hpp
 *
 * Starts node for quadrotor control. Retrieves target location 
 * data and calculates according roll and pitch PID command. Roll
 * command is based on X-axis position of the target and pitch is
 * based on the area of the object.
 *
 * Publishes target command to /ardrone/cmd_vel
 *
 *****************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <cloudora/Location.h>

class controller {
	
	private:
		// Initialize node handle and publishers/subscribers
		ros::NodeHandle n;
		ros::Subscriber location_sub;
		ros::Publisher twist_pub;
		ros::Publisher error_pub;

		// Current position information
		cloudora::Location currentPosition;

		// Acceptable region for target (lateral)
		int x_max;
		int x_min;

		// Acceptable region for target (forward)
		int area_min;
		int area_max;
		int area_threshold; // To make sure it doesn't pick up random blobs as target object

		// Integrator saturation limit
		int integ_sat_roll;
		int integ_sat_pitch;

		// PID gains for lateral (roll)
		double Kp_roll;
		double Ki_roll;
		double Kd_roll;

		// PID gains for forward (pitch)
		double Kp_pitch;
		double Ki_pitch;
		double Kd_pitch;
	
	public:
		controller();

		// Loop
		void loop();

		// Callback for new location information
		void controllerCallBack(const cloudora::Location loc);

};
