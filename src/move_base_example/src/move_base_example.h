#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "pid.h"
#include <string>
#include <cstring>

class MoveBase{
public:
	MoveBase() = delete;
	MoveBase(int, char**,double, double, double);
	~MoveBase();
	void move_to( double,double);

	// ROS variables
  	ros::NodeHandle nh;
	ros::Subscriber sub;
  	ros::Publisher pub;
	void odomcb(const nav_msgs::Odometry::ConstPtr&);
private:
	// Initial Position
	double initx;
	double inity;
	double initth;

	// Current Position
	double currx;
	double curry;
	double currth;
	
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
};




MoveBase::MoveBase(int argc, char **argv, double _initx, double _inity, double _initth){
	// Set up Odometry Subscriber and cmd_vel Publisher
	ros::init(argc, argv, "odom_listener");
	sub = nh.subscribe("odom", 1000, &MoveBase::odomcb, this);
	pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);   

	// Construct PID objects
	linearx = new PID(&linearx_set, &linearx_in, &linearx_out, 
						Kp, Ki, Kd, AUTOMATIC);
	lineary = new PID(&lineary_set, &lineary_in, &lineary_out, 
						Kp, Ki, Kd, AUTOMATIC);
	angularz = new PID(&angularz_set, &angularz_in, &angularz_out, 
						Kp, Ki, Kd, AUTOMATIC);

	// Set Initial and Current position to values passed into constructor
	initx  = currx  = _initx;
	inity  = curry  = _inity;
	initth = currth = _initth;
}

MoveBase::~MoveBase(){
	// Clean Dyanmic Memory allocated by Object
	delete linearx;
	delete lineary;
	delete angularz;
}

double setDigits(double _number, int _digits)
{
    double tenth = pow((double)10,_digits);
    _number *= tenth;
    _number = floor(_number);
    _number /= tenth;

    return _number;
}

void
MoveBase::move_to(double destx, double desty){
	ros::Rate loop_rate(10);

	// While we are not within 2 decimals places of
	// the destination continue running
	while(ros::ok() && 
		setDigits(currx,2) != destx && 
			setDigits(curry,2) != desty){
		// Calculate the normal vector that points in the
		// direction of the destination
		double dx = (destx - currx);
		double dy = (desty - curry);
		double normy = dx / sqrt(pow(dx,2) + pow(dy,2));
		double normx = dy / sqrt(pow(dx,2) + pow(dy,2));
		
		// normx/normy is a the proportion of movement required in
		// respective direction
		linearx_set = normx * scale_factor;
		lineary_set = normy * scale_factor;
		angularz_set  = atan2(normx, normy) * 180 / 3.141592;

		// Compute new output
		linearx->Compute();
		lineary->Compute();
		angularz->Compute();
		
		// Publish new output to cmd_vel
		geometry_msgs::Twist msg;
		msg.linear.x  = linearx_out;
		msg.linear.y  = lineary_out;
		msg.angular.z = angularz_out; 
		pub.publish(msg);

		// Get new messages
		ros::spinOnce();
    		loop_rate.sleep();
	}
}


void MoveBase::odomcb(const nav_msgs::Odometry::ConstPtr& msg)
{
	// Update current position and orientation
	currx  = (msg->pose.pose.position.x);
	curry  = (msg->pose.pose.position.x);
	currth = (msg->pose.pose.orientation.z);
	
	// Update linearx, lineary, and linearth velocities
	linearx_in  = msg->twist.twist.linear.x;
	lineary_in  = msg->twist.twist.linear.y;
	angularz_in = msg->twist.twist.angular.z;
}


