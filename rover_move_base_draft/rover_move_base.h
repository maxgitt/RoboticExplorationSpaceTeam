#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "PID_v1.h"

struct Odometry{
	double x;
	double y;
	double th;

	double dx;
	double dy;
	double dth;
};

class RoverMoveBase {
public:
	RoverMoveBase::RoverMoveBase();

	ros::Subscriber waypoint_sub;
	ros::Subscriber odometry_sub;
	ros::Publisher cmd_vel_pub;

private:
	ros::NodeHandle nh;
	//add waypoint when it has been added
	void waypointCallback(*********);
	void odometryCallback(*********);

	std::pair<double, double> goal;

	//needed for keep track of current linear.x
	double current_velocity;

	double target_angle;
	double target_speed;

	Odometry odometry;
	geometry_msgs::Twist vel;
	//Define the aggressive and conservative Tuning Parameters
	double aggKp=4, aggKi=0.2, aggKd=1;
	double consKp=1, consKi=0.05, consKd=0.25;

	PID anglePID(&odometry.th, &vel.angular.z, &target_angle, consKp, consKi, consKd, DIRECT);
	PID speedPID(&current_velocity, &vel.linear.x, &target_speed, consKp, consKi, consKd, DIRECT);
	//PID speedPID()
};

