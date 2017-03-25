#include <iostream>
#include "ros/ros.h"
#include "rover_particle_filter/Filter.h"
int 
main(int argc, char** argv){
	ros::init(argc, argv, "rover_particle_filter");
	Filter pf(100);
	while(ros::ok()) {
		pf.process();
		ros::spinOnce(); 
	}
}
