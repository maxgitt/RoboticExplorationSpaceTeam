#include <ros/ros.h>

//include go to excavation service
//include excavate service
//include go to sieve service

//include advertise autonomy vs teleop service
//include 


class RoverBrain{
public:
	//Constructor for Rover Brain class 
	RoverBrain();
	MovetoExcavate();
	Excavate();
	MovetoSieve();
	//Called when autonomy_toggle service gets called:
	AutonomyToggle();

private:
	ros::ServiceClient move_to_excavate; //= n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
	ros::ServiceClient excavate;
	ros::ServiceClient move_to_sieve;

	ros::ServiceServer autonomy_toggle;

};