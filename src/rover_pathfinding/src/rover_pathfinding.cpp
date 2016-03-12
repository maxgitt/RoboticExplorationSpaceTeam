#include "ros/ros.h"
#include "rover_pathfinding.h"

Pathfinding_node::Pathfinding_node(){
  global_x = 0;
  global_y = 0;
	sieve_x = 0;
  sieve_y = 0;
  position_iterator = 0;
}

Pathfinding_node::Pathfinding_node(double global_x_input, double global_y_input){
    global_x = global_x_input;
    global_y = global_y_input;
    //ros::NodeHandle nh;
    //ros::Publisher chatter_pub = nh.advertise<rover_pathfinding::waypoint msg>("move_base", 1000);
}

/*
void Pathfinding_node::odometryCallback(const **********::**********:: lastthing) {
  
    //Update local position
  

  current_x = lastthing->x_coord;
  current_y = lastthing->y_coord;
  std::stringstream x_msg_pub, y_msg_pub;
  ros::NodeHandle waypoint_pub;
  ros::Publisher chatter_pub = waypoint_pub.advertise<std_msgs::float64>("move_base", 1000);
  std_msgs::float64 msg;
 
  
  rover_pathfinding::waypoint msg;

  if(position_iterator == 0){
    msg.x = pathway[position_iterator].x_coord;
    msg.y = pathway[position_iterator].y_coord; 
    nh.publish(msg); 
    
    ++position_iterator;
  }
  if(abs(current_x - pathway[position_iterator-1].x_coord) < .1 && abs(current_y - pathway[position_iterator -1].y_coord) < .1){
    
    msg.x = pathway[position_iterator].x_coord;
    msg.y = pathway[position_iterator].y_coord; 
    nh.publish(msg);
      
    ++position_iterator;
  }
}
*/

double Pathfinding_node::get_current_x(){
  return current_x;
}

double Pathfinding_node::get_current_y(){
  return current_y;
}

double Pathfinding_node::get_global_x() {
  return global_x;
}

double Pathfinding_node::get_global_y() {
  return global_y;
}

double Pathfinding_node::get_sieve_x() {
  return sieve_x;
}

double Pathfinding_node::get_sieve_y() {
  return sieve_y;
}

bool Pathfinding_node::get_rover_in_obstacle() {
  return rover_in_obstacle;
}

bool Pathfinding_node::get_rover_in_mining() {
  return rover_in_mining;
}

bool Pathfinding_node::get_rover_in_start() {
  return rover_in_start;
}

void Pathfinding_node::set_global_x(double x) {
  assert(x < 1.89 && x > -1.89);
  assert(global_y < 7.38);
  assert(global_x < 1.89);
  assert(global_x > -1.89);

  global_x = x;
}

void Pathfinding_node::set_global_y(double y) {
  assert(y < 7.38);
  assert(global_y < 7.38);
  assert(global_x < 1.89);
  assert(global_x > -1.89);

  global_y = y;
}
void Pathfinding_node::set_current_x(double x) {
  current_x = x;
}
void Pathfinding_node::set_current_y(double y) {
  current_y = y;
}
void Pathfinding_node::set_rover_in_obstacle(bool a) {
  rover_in_obstacle = a;
}
void Pathfinding_node::set_rover_in_mining(bool a) {
  rover_in_mining = a;
}
void Pathfinding_node::set_rover_in_start(bool a) {
  rover_in_start = a;
}

void Pathfinding_node::set_zone_and_broadcast(double current_y){

  if(current_y < 1.5 && rover_in_start == 0){

        ROS_INFO_STREAM("Rover has entered start area.");
        rover_in_obstacle = 0;
        rover_in_mining = 0;
        rover_in_start = 1;
      }

      if(current_y> 1.5 && rover_in_obstacle == 0){

        ROS_INFO_STREAM("Rover has entered obstacle area.");
        rover_in_obstacle = 1;
        rover_in_start = 0;
        rover_in_mining = 0;
      }


      if(current_y > 4.44 && rover_in_mining == 0){

        ROS_INFO_STREAM("Rover has entered mining area.");
        rover_in_mining = 1;
        rover_in_obstacle = 0;
        rover_in_start = 0;
      }
}

//Function that creates local goal coordinates
void Pathfinding_node::local_goal_creation(){

  double x_new = current_x;
  double y_new = current_y;
  double dist_y = global_y - (current_y);
  double dist_x = global_x - (current_x);
  double steps_y = dist_y / .5;
  double steps_x = dist_x / .5;
  int steps = 0;
  if (steps_y > steps_x) {
    steps = ceil(steps_y);
  }
  else {
    steps = ceil(steps_x);
  }

  //create vector of local goals to end at global goal in set amount of steps
  //UPDATE WHILE CONDITIONS to include buffer?
  while((abs(global_x - current_x) <= .1) && abs(global_y - current_y) <= .1){
    double dist_add_x = dist_x / steps;
    x_new = x_new + dist_add_x;

    double dist_add_y = dist_y / steps;
    y_new = y_new + dist_add_y;

    pathway.push_back(Coordinates(x_new, y_new));
  }
}
