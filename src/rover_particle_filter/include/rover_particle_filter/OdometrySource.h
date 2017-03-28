#ifndef ODOMETRYSOURCE_H
#define ODOMETRYSOURCE_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <string>

template <class MessageType>
class OdometrySource {
public:
	OdometrySource(std::string);
	~OdometrySource();
	typename MessageType::ConstPtr getData();
	
private:
	void odometryCallback(const typename MessageType::ConstPtr&);

	// Node Handle
	ros::NodeHandle nh;

	// Subscriber Handle
	ros::Subscriber sh;

	// User defined topic name
	std::string topic_name;

	typename MessageType::ConstPtr msg;

};

template <class MessageType>
OdometrySource<MessageType>::OdometrySource(std::string topic_name) {
	sh = nh.subscribe<MessageType>(topic_name, 1, & OdometrySource::odometryCallback, this);
}

template <class MessageType>
void OdometrySource<MessageType>::odometryCallback(const typename MessageType::ConstPtr& _msg) {
	msg = _msg;
}

template <class MessageType>
typename MessageType OdometrySource<MessageType>::getData() {
	return *msg;
}

#endif