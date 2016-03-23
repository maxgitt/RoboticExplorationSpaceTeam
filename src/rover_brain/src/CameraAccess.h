#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class CameraAccess{
private:
	bool listening = false;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber sub; 
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
public:
	CameraAccess(int argc, char **argv);
	void getFrame();
};

CameraAccess::CameraAccess(int argc, char **argv): it(nh) {
	// Create image listener node
	ros::init(argc, argv, "image_listener");
	// Create window to view image frames
	cv::namedWindow("view");
	// Launch thread to run window
	cv::startWindowThread();
	// Subscribe to kinect image message
	sub = it.subscribe("rgb/image_raw", 1, &CameraAccess::imageCallback, this);
}

void CameraAccess::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if(!listening) return;
	listening = false;	
	try
	{
        // Convert ROS image tpye to CV image type and display in viewer
		cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}

}

void
CameraAccess::getFrame(){
	// Set listening to true so that callback
	listening = true;
	while(listening){}
}
