#include <iostream>
#include "ros/ros.h"
#include "dig_control.h"


using namespace std;

bool responseFunction(rover_excavation::finish_dig::Request &req, rover_excavation::finish_dig::Response &res)
{	
	dig_control digger;

	if(req.action == "dig")
	{
    	res.finish = digger.digSubRoutine();
	}
	else if(req.action == "dump")
	{
		res.finish = digger.dumpSubRoutine();
	}
	else
	{
		res.finish = false;
		ROS_INFO_STREAM("Invalid action parameter for dig controller");
	}
    return true;
}

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "rover_excavation_node");

		ros::NodeHandle nh;



		ros::ServiceServer dig_service = nh.advertiseService("excavate", responseFunction);

    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::Subscriber cmd_vel_sub;

    return 0;
}
