/* 
 * File:   Odometry.h
 * Authors: pascualy
 *          Kishore B. Rao  Cell: 508-873-5384
 *          Bhairav Metha
 * Description: a class that detects and intializes beacon pairs. calculates pose and position.
 *
 * Created on December 21st, 2015, 4:20 PM
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <Beacon/Beacon.h>
#include <string>
#include <vector>
#include <sstream>
#include <stdlib.h>
#include <cstdio>
#include <unordered_map>
#include <unordered_set>



#ifndef ODOMETRY_H
#define ODOMETRY_H

using std::cin; using std::cout; using std::endl; using std::cerr;
using std::stringstream; using std::istringstream; using std::string;
using std::unordered_map; using std::unordered_set;
using std::pair;

#define NODENAMESPACE 

class Odometry{
public:
	Odometry();
	//detects the beacons which are connected to the computer
	void detectRoverBeacons(); 
	void loadBiases();
	std::pair<double,double> getPosition();
	double getPose();
	void updateOdometry();
	void updateBeaconReadings();

private:
	vector <RoverBeacon> RoverBeacons;
	// used to efficiently update the readings
	unordered_map <string, RoverBeacon*> RoverBeaconsMap;

	pair<double,double> roverPosition;
	double roverPose; //degrees rotation with respect to positive x-axis
    
	// helper function to calculate theta...takes in two pairs of doubles 
	// representing xy positons of two points and determines angle
	double calcAngle(pair<double,double> firstPos, 
		pair<double,double> secondPos);


	ros::NodeHandle nh; //creates node handle
	//will allow this class to publish data for other nodes to listen to
	ros::Publisher odom_pub; 

	//tf::TransformBroadcaster odom_broadcaster;
	geometry_msgs::Quaternion odom_quat;
	geometry_msgs::TransformStamped odom_trans;

	ros::Time current_time;
	ros::Time last_time;


	// Odometry variables
	double x_prev = 0;
	double y_prev = 0;
	double th_prev = 0;
    
	double x = 0;  //x position
	double y = 0;  //y position
	double th = 0; //angle of
    
	double vx = 0;
	double vy = 0;
	double vth = 0;

	int numSieveBeacons = 0;
};

const static std::string param_key = "/rover_odometry/";

#endif
