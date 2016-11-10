/* This header file contains all the code to access the camera
 * on the rover. To get this running include this header in your project
 * and construct an the CameraAccess. Once the freenect_camera_node
 * or any other image publishing node is running, a call to getFrame
 * will produce an image frame in the viewer.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/callback_queue.h>

class CameraAccess {
private:
	ros::NodeHandle *nh = nullptr;
	image_transport::ImageTransport *it = nullptr;
	image_transport::Subscriber sub; 
	ros::CallbackQueue image_callback_queue;
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
public:
	CameraAccess(int argc, char **argv);
	void getFrame();
};

CameraAccess::CameraAccess(int argc, char **argv) {
	// Initialize camera access node
	ros::init(argc, argv, "image_listener");
	// Create image listener node handle
	nh = new ros::NodeHandle();
	// Detach image message queue from node and assign to our custom one;
	nh->setCallbackQueue(&image_callback_queue);
	// Create image listener node
	it = new image_transport::ImageTransport(*nh);
	// Create window to view image frames
	cv::namedWindow("view");
	// Launch thread to run window
	cv::startWindowThread();
	// Subscribe to kinect image message
	sub = it->subscribe("rgb/image_raw", 1, &CameraAccess::imageCallback, this);
}

void CameraAccess::imageCallback(const sensor_msgs::ImageConstPtr& msg) {	
	try {
        	// Convert ROS image tpye to CV image type and display in viewer
		cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
		cv::waitKey(30);
	}catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void
CameraAccess::getFrame() {
	// Process one image message
	image_callback_queue.callOne(ros::WallDuration());
}
