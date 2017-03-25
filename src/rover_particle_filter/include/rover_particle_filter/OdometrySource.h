#ifndef ODOMETRYSOURCE_H
#define ODOMETRYSOURCE_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <string>

class OdometrySource {
public:
	OdometrySource(std::string);
	~OdometrySource();
	void getData(nav_msgs::Odometry::ConstPtr&);
	
private:
	void odometryCallback(const nav_msgs::Odometry::ConstPtr&);

	// Node Handle
	ros::NodeHandle nh;

	// Subscriber Handle
	ros::Subscriber sh;

	// User defined topic name
	std::string topic_name;

	nav_msgs::Odometry::ConstPtr msg;

};

OdometrySource::OdometrySource( std::string topic_name) {
	sh = nh.subscribe<nav_msgs::Odometry>(topic_name, 1, &OdometrySource::odometryCallback, this);
}

void
OdometrySource::odometryCallback(const nav_msgs::Odometry::ConstPtr& _msg) {
	msg = _msg;
}

void 
OdometrySource::getData(nav_msgs::Odometry::ConstPtr& _msg) {
	_msg = msg;
}

#endif