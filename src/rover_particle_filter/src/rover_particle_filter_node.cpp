#include <iostream>
#include "ros/ros.h"
#include "rover_particle_filter/MCFilter.h"
int 
main(int argc, char** argv){
	ros::init(argc, argv, "rover_particle_filter");

	MCFilter::modelParam newModel;
	MCFilter pf(100, newModel, MCFilter::SelectionAlgorithm_t::ROBUST_MEAN);
	while(ros::ok()) {
		pf.process();
		ros::spinOnce(); 
	}
}
