#ifndef ODOMETRYDESTINATION_H
#define ODOMETRYDESTINATION_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <string>

template <class MessageType>
class OdometryDestination {
public:
	OdometryDestination(std::string);
	~OdometryDestination();
	void setData(const typename MessageType::ConstPtr&);
	
private:
	// Node Handle
	ros::NodeHandle nh;

	// Subscriber Handle
	ros::Publisher ph;

	// User defined topic name
	std::string topic_name;

};

template <class MessageType>
OdometryDestination<MessageType>::OdometryDestination(std::string topic_name) {
	ph = nh.advertise<MessageType>(topic_name, 20);
}

template <class MessageType>
void OdometryDestination<MessageType>::setData(const typename MessageType::ConstPtr& _msg) {
	ph.publish(_msg);
}

#endif