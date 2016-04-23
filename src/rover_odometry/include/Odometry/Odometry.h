/* 
 * File:   Odometry.h
 * Authors: pascualy
 *          Kishore B. Rao  Cell: 508-873-5384
 *          Bhairav Mehta
 * Description: a class that detects and intializes beacon pairs. calculates pose and position.
 *
 * Created on December 21st, 2015, 4:20 PM
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <Beacon/Beacon.h>
#include <string>
#include <vector>
#include <sstream>
#include <cstdio>
#include <unordered_map>


#include <iostream>
#include <sys/resource.h>
#include <sys/time.h>

#ifndef ODOMETRY_H
#define ODOMETRY_H

class Timer {
private:
     struct rusage startu;
     struct rusage endu;
     double duration;
public:
     Timer() { getrusage(RUSAGE_SELF, &startu); }
    
     void recordTime(){
         getrusage(RUSAGE_SELF, &endu);
         double start_sec = startu.ru_utime.tv_sec + startu.ru_utime.tv_usec/1e7;
         double end_sec = endu.ru_utime.tv_sec +endu.ru_utime.tv_usec/1e7;
         duration = end_sec - start_sec;
         } // recordTime()
    
        double getTime() { return duration; }
};


class Odometry{
public:
	Odometry();
	//Detect the number of rover beacons connected to the rover
	void detectRoverBeacons();
    //Load the biases either manually or through the parameter server
	void loadBiases();
    //Return the 
	std::pair<double,double> getPosition();
	double getPose();
	void updateOdometry();
	void updateBeaconReadings();

private:
	std::vector <RoverBeacon> RoverBeacons;
	// used to efficiently update the readings
	std::unordered_map<std::string, RoverBeacon*> RoverBeaconsMap;

	std::pair<double,double> roverPosition;
	double roverPose; //degrees rotation with 
			  //respect to positive x-axis
    
	// helper function to calculate theta...takes in two pairs 
	// of doublesrepresenting xy positons of two points and 
	// determines angle
	double calcAngle(std::pair<double,double> firstPos, 
				std::pair<double,double> secondPos);


	ros::NodeHandle nh; //creates node handle
	//will allow this class to publish data for other nodes to listen to
	ros::Publisher odom_pub; 
	void publish_odometry();

  	tf::TransformBroadcaster odom_broadcaster;

	ros::Time current_time;
	ros::Time last_time;
	bool first = true;
	// Odometry variables
	double x_prev = 0;
	double y_prev = 0;
	double th_prev = 0;
    
	double x = 0;  //x position
	double y = 0;  //y position
	double th = 0; //angle of
    
	//time between odometry calculation
	double dt = 0;
	
	//velocites in linear x, linear y, and angular z
	double vx = 0;
	double vy = 0;
	double vth = 0;

	int numSieveBeacons = 0;
	std::deque <double> runningTotal;
	int currCount;
    
    	Timer timer;
};

const static std::string param_key = "/rover_odometry/";

#endif
