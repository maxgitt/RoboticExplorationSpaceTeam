//Pathfinding Node
//Adithya Ramanathan & Rishi Bhuta
//REST 2015 - S3FL
#include <cmath>
#include "stdio.h"
#include "ros/ros.h"
#include "rover_pathfinding/global_goal.h"
#include "std_msgs/Float64.h"
#include "move_base.h"
#include <geometry_msgs/Twist.h>
#include "pid.h"

using namespace std;

int main(int argc, char** argv){
    //Broadcast a simple log message 
    ROS_INFO_STREAM("Pathfinding has begun.");
    MoveBase rover_mover(argc, argv);
    ros::Rate loop_rate(10);
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}