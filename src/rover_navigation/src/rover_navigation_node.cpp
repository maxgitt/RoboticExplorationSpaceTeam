#include <iostream>
#include "ros/ros.h"

#include "rover_navigation/Navigation.h"

int 
main(int argc, char** argv){
	ros::init(argc, argv, "rover_navigation");
	Navigation nav;
	while(ros::ok()) {
		nav.process();
		ros::spinOnce(); 
	}
}
