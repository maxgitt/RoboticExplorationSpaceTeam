#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <string>
#include <stdio.h>
#include <sstream>
#include <iostream>
#include "Beacon.h"
#include "Odometry.h"


int main(){
	//Initialize ROS node and Odometry Publisher
	ros::init(argc, argv, "rover_odometry");
	ros::NodeHandle nh;
	odometryPublisher = nh.advertise<nav_msgs::Odometry>("odom",50);
	Odometry::Odometry rover_odometry;

	rover_odometry.detectBeacons(); //detects the beacons which are connected to the computer
	rover_odometry.initializeBeacons();
	rover_odometry.calculateBiases();

	ros::Rate r(1.0);
   	while(ros::ok()){
   		rover_odometry.updateOdometry();
   		r.sleep();
   	}
}