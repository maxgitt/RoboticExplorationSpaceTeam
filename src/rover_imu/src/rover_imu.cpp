#include "imu_control.h"
#include "serial/serial.h"
#include <iostream>

int 
main(int argc, char** argv){
	ros::init(argc, argv, "rover_imu");
	IMUControl imu_controller;
	while(ros::ok()) {
		imu_controller.receive();
		ros::spinOnce(); 
		ros::Duration(.1).sleep();
	}
}

