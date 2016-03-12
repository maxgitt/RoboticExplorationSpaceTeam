#include "ros/ros.h"
#include "RoverBrain.h"

RoverBrain::RoverBrain() : auto_off_flag(false) {
	move = nh.serviceClient<rover_pathfinding::global_goal>("pathfinding");
	excavate = nh.serviceClient<rover_excavation::finish_dig>("excavate");

	autonomy_toggle = nh.advertiseService("autonomy_toggle", &RoverBrain::AutonomyToggle,this);
}

void RoverBrain::MovetoExcavate(){
	if (auto_off_flag) return;
	rover_pathfinding::global_goal m2e; // create new srv file
	
	// call param server to fill request fields
	nh.param("global_goal_x", m2e.request.x);
	nh.param("global_goal_y", m2e.request.y);           
	
	if (move.call(m2e)){ // call service with msg
		//ROS_INFO("Calling pathfinding to %d, %d", (float)m2e.request.x, (float)m2e.request.y);
		if (m2e.response.at_dest) std::cout << "Excavate" << std::endl; Excavate();
	}
	else {
		ROS_ERROR("Failed to call service move_to_dest for excavation site");
		return;
	}
}

void RoverBrain::Excavate(){
	if (auto_off_flag) return;
	rover_excavation::finish_dig exc; // create new srv file

	if (excavate.call(exc)){ // call service with msg
		//ROS_INFO("Calling excavation");
		if (exc.response.finish) std::cout << "MovetoSieve" << std::endl; MovetoSieve();
	}
	else {
		ROS_ERROR("Failed to call service excavate for excavation");
		return;
	}
}

void RoverBrain::MovetoSieve(){
	if (auto_off_flag) return;
	rover_pathfinding::global_goal m2s; // create new srv file

	// call param server to fill request fields
	nh.param("global_goal_x", m2s.request.x);
	nh.param("global_goal_y", m2s.request.y);

	if (move.call(m2s)){ // call service with msg
		//ROS_INFO("Calling pathfinding to %d, %d", (float)m2s.request.x, (float)m2s.request.y);
		if (m2s.response.at_dest) std::cout << "Deposit" << std::endl; Deposit();
	}
	else {
		ROS_ERROR("Failed to call service move_to_dest for sieve");
		return;
	}
}

void RoverBrain::Deposit(){
	if (auto_off_flag) return;
	rover_excavation::finish_dig dep; // create new srv file

	if (excavate.call(dep)){ // call service with msg
		//ROS_INFO("Calling excavation");
		if (dep.response.finish) std::cout << "MovetoExcavate" << std::endl; MovetoExcavate();
	}
	else {
		ROS_ERROR("Failed to call service excavate for depositing");
		return;
	}
}

bool RoverBrain::AutonomyToggle(rover_brain::auto_togg::Request &req, rover_brain::auto_togg::Response &res){
	auto_off_flag = res.auto_off = req.turn_off;
	return true;
}
