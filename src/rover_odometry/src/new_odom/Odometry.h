/* 
 * File:   Odometry.h
 * Authors: pascualy
 *          Kishore B. Rao  Cell: 508-873-5384
 *          Bhairav Metha
 * Description: a class that detects and intializes beacon pairs. calculates pose and position.
 *
 * Created on December 21st, 2015, 4:20 PM
 */
//#include <ros/ros.h>
//#include <tf/transform_broadcaster.h>
//#include <nav_msgs/Odometry.h>
#include <string>
#include <vector>
#include <sstream>
#include "Beacon.h"

#ifndef ODOMETRY_H
#define ODOMETRY_H



#define DRIVER_PREFIX "script -c \"sudo '/home/pascualy/catkin_ws/decawave_driver/dwm1000driver'"
#define DRIVER_POSTFIX "\" /dev/null"


class Odometry{
public:
	Odometry();
	void detectRoverBeacons(); //detects the beacons which are connected to the computer
    void loadBiases();
	std::pair<double,double> getPosition();
	double getPose();
	void updateOdometry();

private:
    FILE* driverData;

    std::vector <RoverBeacon> RoverBeacons;

	std::pair<double,double> roverPosition;
	double roverPose; //degrees rotation with respect to positive x-axis
    
    //helper function to calculate theta...takes in two pairs of doubles representing xy positons of two points and determines angle
    double calcAngle(std::pair<double,double> firstPos, std::pair<double,double> secondPos);


    ros::NodeHandle nh; //creates node handle
	ros::Publisher odom_pub; //will allow this class to publish data for other nodes to listen to

    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_trans;

    ros::Time current_time;
    ros::Time last_time;


    //Odometry variables
    double x_prev = 0;
    double y_prev = 0;
    double th_prev = 0;
    
    double x = 0;  //x position
    double y = 0;  //y position
    double th = 0; //angle of
    
    double vx = 0;
    double vy = 0;
    double vth = 0;

};


#endif