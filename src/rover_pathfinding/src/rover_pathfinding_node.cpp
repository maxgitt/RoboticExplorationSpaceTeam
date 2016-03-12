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
//Adithya Ramanathan & Rishi Bhuta
//REST 2015 - S3FL
#include <cmath>
#include <stdio.h>
#include "ros/ros.h"
#include "rover_pathfinding/global_goal.h"
#include "rover_pathfinding.h"
#include "std_msgs/Float64.h"
#include "rover_odometry/Odom.h"
#include <geometry_msgs/Twist.h>
#include "PID_v1.h"
using namespace std;

bool pathfinding(rover_pathfinding::global_goal::Request &req, rover_pathfinding::global_goal::Response &res);

int main(int argc, char** argv){

  // Announce this program to the master as a node called pathfinding
  ros::init(argc, argv, "pathfinding_node");
  ros::NodeHandle nh;

  //Broadcast a simple log message 
  ROS_INFO_STREAM("Pathfinding has begun.");
     
  ros::ServiceServer service = nh.advertiseService("pathfinding", &pathfinding);
  ros::Rate loop_rate(10);
  while (ros::ok()){
  	ros::spinOnce();
	loop_rate.sleep();
  }
  
  //Stops the node resources
  //ros::spin();

  return 0;

}


bool pathfinding(rover_pathfinding::global_goal::Request &req, rover_pathfinding::global_goal::Response &res){
      //Create Instance of Pathfinding_node Class
      Pathfinding_node path;
      //res.message = "We have arrive \n";
      //ROS_INFO("request: x=%id, y=%id", (float64)req.x, (float64)req.y);
	res.at_dest = true;
     return true;
      double global_x = req.x; //Get from service
      double global_y = req.y; //Get from service
     while(ros::ok())  {
        
          //update speed

          //update angle

        rover_odometry::Odom odometry;
        geometry_msgs::Twist vel;
        //Define the aggressive and conservative Tuning Parameters
        double aggKp=4, aggKi=0.2, aggKd=1;
        double consKp=1, consKi=0.05, consKd=0.25;
/*
        PID anglePID(&odometry.th, &vel.angular.z, &target_angle, consKp, consKi, consKd, DIRECT);
        PID speedPID(&current_velocity, &vel.linear.x, &target_speed, consKp, consKi, consKd, DIRECT);

         while(abs(global_x - path.get_current_x()) > .1 && abs(global_y - path.get_current_y()) > .1){ //Check that we're at the global goal
            //Set Global Variables
            path.set_global_x(global_x);
            path.set_global_y(global_y);
                           
            anglePID.Compute();
            speedPID.Compute();
            //Broadcast message relating to where the rover is.                  
            //path.set_zone_and_broadcast();
              
            //Send the local goal to move_base
            path.local_goal_creation();


          //Send message to brain that we have arrived

          ros::spinOnce;
        }*/
        
      }

  //ROS_INFO("sending back respons: [%id]", (string)res.messaged);
  return true;
}
