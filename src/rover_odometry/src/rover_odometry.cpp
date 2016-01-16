#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <string>
#include <stdio.h>
#include <sstream>
#include <iostream>
#include <Beacon/Beacon.h>
#include <Odometry/Odometry.h>


int main(int argc, char * argv[]){
	//Initialize ROS node and Odometry Publisher
	ros::init(argc, argv, "rover_odometry");
	ros::NodeHandle nh;
	//ros::Publisher odometryPublisher = nh.advertise<nav_msgs::Odometry>("odom",50);
	Odometry rover_odometry;

	rover_odometry.detectRoverBeacons(); //detects the beacons which are connected to the computer
	rover_odometry.loadBiases();

	ros::Rate r(1.0);
   	while(ros::ok()){
   		rover_odometry.updateOdometry();
   		r.sleep();
   	}
}
