#include "ros/ros.h"

#include <vector>
#include <string>

class PoseArray
{
public:
	PoseArray();
	~PoseArray();
	void publish(const std::vector<Particle>&);
private:
	// Node Handle
	ros::NodeHandle nh;

	// Publisher Handle
	ros::Publisher ph;
};

PoseArray::PoseArray(std::string topic_name) {
	ph = nh.advertise<nav_msgs::Odometry>(topic_name, 50);
}

void
PoseArray::publish(const std::vector<Particle>& particle_set) {
	geometry_msgs::PoseArray cloud_msg;
	cloud_msg.header.stamp = ros::Time::now();
	cloud_msg.header.frame_id = global_frame_id_;
	cloud_msg.poses.resize(particle_set.size());
	for(auto particle: particle_set) {
	    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(particle.getTh()),
	                             tf::Vector3(particle.getX(), particle.getY(), 0)),
	                    										cloud_msg.poses[i]);
  	}
  	ph.publish(cloud_msg);
}