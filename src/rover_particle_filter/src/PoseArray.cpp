#include "rover_particle_filter/PoseArray.h"


PoseArray::PoseArray() {
	nh.param("/rover_particle_filter/pose_array/topic_name", topic_name, topic_name);
	nh.param("/rover_particle_filter/pose_array/global_frame_id", global_frame_id, global_frame_id);
	ph = nh.advertise<geometry_msgs::PoseArray>(topic_name, 5, true);
}

PoseArray::PoseArray(std::string _topic_name, std::string _global_frame_id) {
	topic_name = _topic_name;
	global_frame_id = _global_frame_id;
	ph = nh.advertise<geometry_msgs::PoseArray>(topic_name, 5, true);
}

void
PoseArray::publish(std::vector<Particle>& particle_set) {
	cloud_msg.header.stamp = ros::Time::now();
	cloud_msg.header.frame_id = global_frame_id;
	cloud_msg.poses.resize(particle_set.size());
	for(int i = 0; i < particle_set.size(); ++i) {
	    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(particle_set[i].getTh()),
	                             tf::Vector3(particle_set[i].getX(), particle_set[i].getY(), 0)),
	                    										cloud_msg.poses[i]);
  	}
  	ph.publish(cloud_msg);
}
