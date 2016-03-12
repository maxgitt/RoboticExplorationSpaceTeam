#include <iostream>
#include "ros/ros.h"
#include "dig_control.h"

using namespace std;

bool responseFunction(rover_excavation::finish_dig::Request &req, rover_excavation::finish_dig::Response &res)
{
    res.finish = true;
    return true;
}

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "rover_excavation_node");

		ros::NodeHandle nh;

		dig_control digger;
    digger.digSubRoutine();

		ros::ServiceServer dig_service = nh.advertiseService("excavate", responseFunction);

    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::Subscriber cmd_vel_sub;

    return 0;
}
