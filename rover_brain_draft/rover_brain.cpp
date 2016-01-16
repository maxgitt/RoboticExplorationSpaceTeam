RoverBrain(){
	//types not defined yet
	ros::ServiceClient move_to_excavate = nh.serviceClient<rover_move_base::move_to_excavate>("move_to_excavate");
	ros::ServiceClient excavate 		= nh.serviceClient<rover_escavate::excavate>("xcavate");
	ros::ServiceClient move_to_sieve	= nh.serviceClient<rover_move_base::move_to_sieve>("move_to_sieve");

	ros::ServiceServer autonomy_toggle  = nh.advertiseService("autonomy_toggle", autonomy_toggle);


}

RoverBrain::MovetoExcavate(){



}

RoverBrain::Excavate(){



}

RoverBrain::MovetoSieve(){


}

RoverBrain::AutonomyToggle(rover::AddTwoInts::Request  &req,
					beginner_tutorials::AddTwoInts::Response &res){



}