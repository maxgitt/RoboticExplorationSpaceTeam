/*
filename: rover_pathing/msg/Waypoint.msg

Header header

float64 x
float64 y

filename: CMakeList.txt
  

catkin_package(
## INCLUDE_DIRS include
## LIBRARIES rover_motor_control rover_encoder_control
 CATKIN_DEPENDS roscpp message_runtime
## DEPENDS system_lib
)

  
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Waypoint.msg
)
  
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)
  
  
filename: package.xml

<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>


  rover_pathfinding::Waypoint msg;
  msg.x;
  msg.y
  waypoint_pub.publish(msg);
*/

//Pathfinding Node
//Adithya Ramanathan & Bhairav Mehta & (!Rishi Bhuta)
//REST 2015 - S3FL
#include <cmath>
#include <stdio>
#include <ros/ros.h>
#include <coordinates.h>
// fix this
#include <pathfinding_node.cpp>



using namespace std;


int main(int argc, char** argv){
  
  ros::NodeHandle nh;
  //Change "REST" with package name
	ros::ServiceClient client = nh.serviceClient<REST::BroadcastGlobalGoals>("broadcast_global_goal");
	REST::BroadcastGlobalGoals srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  
  
  
	if (client.call(srv))
	{
		// Announce this program to the master as a node called pathfinding
		ros::init(argc, argv, "pathfinding_node");

		//Broadcast a simple log message 
		ROS_INFO_STREAM("Pathfinding has begun.");
   
		double global_x = srv.a; //Get from services
    double global_y = srv.b; //Get from service
    
     //Create Instance of Pathfinding_node Class
    Pathfinding_node path;
    
    //Set Global Variables
    path.set_global_x(global_x);
    path.set_global_y(global_y);
                   
    //Broadcast message relating to where the rover is.                  
    path.set_zone_and_broadcast();
      
    //Send the local goal to move_base
    path.local_goal_creation();
 
	}

	//Stops the node resources
	ros::shutdown();

	return 0;

}
