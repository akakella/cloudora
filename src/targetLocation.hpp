/******************************************************************
 * Abishek Akella, Cyber-Physical Cloud Computing Laboratory (2012)
 *
 * targetLocation.hpp
 *
 * Starts node for target location. Displays video stream from the
 * forward-facing camera of the Parrot AR.Drone and video stream
 * containing processed (HSV-thresholded) video data.
 *
 * Publishes target location to /ardrone/target_location
 *
 *****************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <cloudora/Location.h>

class targetLocation {

	private:
		// CurrentLocation describes current location of the target
		cloudora::Location currentLocation;

		// Initialize subscriber to video feed and publisher for target location
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;
		ros::Publisher location_pub;

		// Node handle
		ros::NodeHandle n;

	public:
		targetLocation();
		~targetLocation();

		// Callback for new images being acquired from forward-facing camera
		void imageCallBack(const sensor_msgs::ImageConstPtr& original_image);

		// Main loop for continual updating
		void loop();

};
