#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image>
#include <sensor_msgs/image_encodings.h>

#include "segmentation.h"

void depthImageCb(const sensor_msgs::ImageConstPtr& msg){
    // TODO: Subtract the image from the background image -- make this into a class
    
    cv_bridge::CvImagePtr cv_ptr;

    try{
        cv_ptr =  toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }

    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // TODO: Make it a struct object -- x,y, width 
    std::vector<pair<int, int>> obstacles;

    Segmentation::segmentDepthImage(cv_ptr->image, obstacles);

    // TODO: add obstacle publisher here

    return;
}

int main(int argc, char** argv){
    ros::NodeHandle nh;

    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

    // TODO: Fix path
    image_sub = it.subscribe("/camera/depth", 1, &depthImageCb);



}