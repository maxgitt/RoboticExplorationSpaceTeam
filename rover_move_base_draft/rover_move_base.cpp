RoverMoveBase::RoverMoveBase(){
	waypoint_sub = nh.subscribe<*********>("waypoint", 5, &RoverMoveBase::waypointCallback, this);
	cmd_vel_pub  = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	//we must create a custom odom message
	odometry_sub = nh.subscribe<***********>("odom", 100, &RoverMoveBase::odometryCallback, this);

}

RoverMoveBase::waypointCallback(********){
	//assuming waypoint is message variable
	goal.first  = msg->waypoint.x;
	goal.second = msg->waypoint.y;
}

RoverMoveBase::odometryCallback(*********){
	odometry.x   = msg->position.x;
	odometry.y   = msg->position.y;
	odometry.th  = msg->position.th;
	odometry.dx  = msg->velocity.x;
	odometry.dy  = msg->velocity.y;
	odometry.dth = msg->velocity.th;






}