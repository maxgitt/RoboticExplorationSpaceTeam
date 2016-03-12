#include "ros/ros.h"
#include "RoverBrain.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "brain_node");

	RoverBrain rover_brain;

	ros::Rate loop_rate(10);
	while (ros::ok()){
		rover_brain.MovetoExcavate();
	  	ros::spinOnce();
		loop_rate.sleep();
	}

	//ros::spin();
	return(0);
}
