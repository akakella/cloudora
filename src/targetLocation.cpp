/******************************************************************
 * Abishek Akella, Cyber-Physical Cloud Computing Laboratory (2012)
 *
 * targetLocation.cpp
 *
 * Starts node for target location. Displays video stream from the
 * forward-facing camera of the Parrot AR.Drone and video stream
 * containing processed (HSV-thresholded) video data.
 *
 * Publishes target location to /ardrone/target_location
 *
 *****************************************************************/

#include "targetLocation.hpp"

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

namespace encode = sensor_msgs::image_encodings;

// Windows for video stream
static const char WINDOW[] = "Image Processed";
static const char ORIGINAL[] = "AR Drone Video Stream";

targetLocation::targetLocation() : it(n) {
	// Initialize current target location and video stream windows
	currentLocation.posX = 0;
	currentLocation.posY = 0;
	currentLocation.area = 0;
	cvNamedWindow(ORIGINAL);
	cvNamedWindow(WINDOW);
	cvStartWindowThread();

	// Initialize ROS publishers/subscribers
	image_sub = it.subscribe("ardrone/image_raw", 1, &targetLocation::imageCallBack, this);
	location_pub = n.advertise<cloudora::Location>("ardrone/target_location", 1);
}

targetLocation::~targetLocation() {
	// Destroy windows on exit
	cv::destroyWindow(WINDOW);
	cv::destroyWindow(ORIGINAL);
}

void targetLocation::imageCallBack(const sensor_msgs::ImageConstPtr& original_image) {
	ROS_INFO("IN CALLBACK");

	// Set up CvBridge for conversion and try to copy
	cv_bridge::CvImagePtr cv_ptr;
	sensor_msgs::cvBridge bridge;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(original_image, encode::RGB8);
		cvShowImage(ORIGINAL, bridge.imgMsgToCv(original_image, "bgr8"));
	} catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("exception: %s", e.what());
		return;
	}

	// Create IplImage and convert image to HSV
	IplImage* imgHSV = cvCreateImage(cvGetSize(bridge.imgMsgToCv(original_image)), 8, 3);
	cvCvtColor(bridge.imgMsgToCv(original_image), imgHSV, CV_RGB2HSV);
	IplImage* imgThreshed = cvCreateImage(cvGetSize(bridge.imgMsgToCv(original_image)), 8, 1);

	// Perform color thresholding on HSV image to filter out blue
	cvInRangeS(imgHSV, cvScalar(0, 100, 50), cvScalar(17, 255, 255), imgThreshed);
	cvShowImage(WINDOW, imgThreshed);
	cvReleaseImage(&imgHSV);

	// Calculate X and Y moments of thresholded image to determine target location
	CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
	cvMoments(imgthreshed, moments, 1);
	double moment10 = cvGetSpatialMoment(moments, 1, 0);
	double moment01 = cvGetSpatialMoment(moments, 0, 1);

	// Central moment determines target area
	double area = cvGetCentralMoment(moments, 0, 0);

	// Build Location msg and publish target location
	cloudora::Location loc;
	loc.posX = moment10/area;
	loc.posY = moment01/area;
	loc.area = area;
	location_pub.publish(loc);
}

void targetLocation::loop()
{
	// Loop
	ros::spin();
}
