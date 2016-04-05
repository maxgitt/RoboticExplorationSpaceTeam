#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "rover_pathfinding/global_goal.h"
#include "pid.h"
#include <string>
#include <cstring>

struct Coordinate{
	double x;
	double y;
	Coordinate(double _x, double _y) : x(_x), y(_y) {}
};

class MoveBase{
public:
	MoveBase() = delete;
	MoveBase(int argc, char **argv);
	MoveBase(int, char**, double, double, double);
	void setup(int argc, char **argv);
	~MoveBase();
	void move_to( double,double);
	void move_along(std::vector<Coordinate>&);
	void record_path();
	// ROS variables
  	ros::NodeHandle nh;
	ros::Subscriber sub;
  	ros::Publisher pub;
  	ros::ServiceServer srv;
	void odomcb(const nav_msgs::Odometry::ConstPtr&);
	void pathcb(rover_pathfinding::global_goal::Request&,
    			rover_pathfinding::global_goal::Response&);
private:
	// Initial Position
	double initx  = -100;
	double inity  = -100;
	double initth = -100;

	// Current Position
	double currx  = -100;
	double curry  = -100;
	double currth = -100;

	double destx = -100;
	double desty = -100;

	// Distance from destination
	double dist_from_dest;

	double scale_factor = 1;

	// Tuning Params
	double Kp=2, Ki=5, Kd=1;

	// PID controller for linear x
	PID * linearx;
	double linearx_set, linearx_in, linearx_out;

	// PID controller for linear y
	PID * lineary;
	double lineary_set, lineary_in, lineary_out;

	// PID controller for angular z
	PID * angularz;
	double angularz_set, angularz_in, angularz_out;

	std::vector<Coordinate> recorded_path;

	const int dist_from_dest_thresh_c = 2;
	bool at_dest();

};
