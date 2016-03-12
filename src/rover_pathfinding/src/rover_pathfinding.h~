#include "ros/ros.h"
#include "coordinates.h"
#include <vector>

using namespace std;

class Pathfinding_node
{
public:
  Pathfinding_node();
  Pathfinding_node(double global_x_input, double global_y_input);
     
  double get_global_x();
  double get_global_y();
  double get_current_x();
  double get_current_y();
  double get_sieve_x();
  double get_sieve_y();
  
  bool get_rover_in_obstacle();
  bool get_rover_in_mining();
  bool get_rover_in_start();
  void set_global_x(double x);
  void set_global_y(double y);
  void set_current_x(double x);
  void set_current_y(double y);
  void set_rover_in_obstacle(bool a);
  void set_rover_in_mining(bool a);
  void set_rover_in_start(bool a);
  void local_goal_creation();
  void set_zone_and_broadcast(double current_y);
  void broadcast_path();

  //ros::Subscriber ********("Arrived", 10, &odometryCallback, this);

private:

  
  double global_x;
  double global_y;
  double sieve_x;
  double sieve_y;
  double current_x;
  double current_y;
  bool rover_in_obstacle;
  bool rover_in_mining;
  bool rover_in_start;
  //void odometryCallback();
  int position_iterator;
  vector<Coordinates> pathway;

};
