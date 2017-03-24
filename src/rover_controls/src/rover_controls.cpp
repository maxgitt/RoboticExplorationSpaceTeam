#include "drivetrain_control.h"
#include "serial/serial.h"
#include <iostream>

int 
main(int argc, char** argv){
	ros::init(argc, argv, "rover_controls");
	DriveTrainControl drivetrain_controller;
	while(ros::ok()) {
		drivetrain_controller.transmit();
		drivetrain_controller.receive();
		ros::spinOnce(); 
		ros::Duration(.1).sleep();
	}
}

