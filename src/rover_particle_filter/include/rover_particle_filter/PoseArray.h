#ifndef POSEARRAY_H
#define POSEARRAY_H

#include "rover_particle_filter/Particle.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <vector>
#include <string>

class PoseArray
{
public:
	PoseArray();
	PoseArray(std::string, std::string);
	~PoseArray();
	void publish(std::vector<Particle>&);
private:
	// Node Handle
	ros::NodeHandle nh;

	// Publisher Handle
	ros::Publisher ph;

	// Publishing Data
	geometry_msgs::PoseArray cloud_msg;
	std::string topic_name;
	std::string global_frame_id;
};

#endif 	