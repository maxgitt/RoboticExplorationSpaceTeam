#include "rover_camera.h"
#include "iostream"
int main(int argc, char **argv){
	// Create camera access object and request
	// a single frame
	CameraAccess camera(argc, argv);
	while(ros::ok()){
		camera.getFrame();
		ros::Duration(5).sleep();
	}
	return 0;
}
