#include "ros/ros.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <cmath>

//serial setup
#define BAUD 9600
#define PORT "/dev/ttyACM1"

//rover physical properties
#define ROBOT_WIDTH 1 //meters
#define WHEEL_DIAMETER .5 //meters

using namespace std;

class RoverMotorControl {

public:
	RoverMotorControl();
	~RoverMotorControl();
	double getvl();
	double getvr();
	serial::Serial * uno_serial;
private:
	void callback(const geometry_msgs::Twist &twist_aux);
	double vl_;
	double vr_;
	int linear_, angular_;
	double l_scale_, a_scale_;
	double robot_width_, wheel_diameter_;

	ros::NodeHandle nh;
	ros::Subscriber cmd_vel_sub; 
};

void RoverMotorControl::callback(const geometry_msgs::Twist &twist_aux){
	double vel_x = twist_aux.linear.x;
	double vel_th = twist_aux.angular.z;
	double right_vel = 90;
	double left_vel = 90;
	if(abs(vel_x) == 0){
		// turning
		right_vel = vel_th * ROBOT_WIDTH / 2.0 * 90 + 90;
		left_vel = 180 - right_vel;
	}else if(abs(vel_th) == 0){
		// forward / backward
		left_vel  = vel_x * 90 + 90;
		right_vel = vel_x * 90 + 90;
	}else{
		// moving doing arcs
		left_vel = (vel_x - vel_th * ROBOT_WIDTH / 2.0) * 90 + 90;
		right_vel = (vel_x + vel_th * ROBOT_WIDTH / 2.0) * 90 + 90;
	}
	vl_ = left_vel;
	vr_ = right_vel;
}

//Initialize serial port 
RoverMotorControl::RoverMotorControl()
{
	vl_ = 90;
	vr_ = 90;
	try {
		uno_serial = new serial::Serial(PORT, BAUD, serial::Timeout::simpleTimeout(1000));
	}
	catch(serial::IOException e){
		std::cerr << "ERROR connecting to serial" << std::endl;
		exit(0);
	}
	cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &RoverMotorControl::callback, this);
}

RoverMotorControl::~RoverMotorControl(){
	delete uno_serial;
}

double RoverMotorControl::getvl()
{
	return vl_;

}

double RoverMotorControl::getvr()
{
	return vr_;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "motor_control_rest"); //initializes the node 
	RoverMotorControl controller;
	ros::Rate loop_rate(10000); //only run while loop every 10 milliseconds 
	while(ros::ok())
	{	
		stringstream ss; 
		ss << controller.getvl() << "," << controller.getvr() << "," << controller.getvl() << "," << controller.getvr() << "\n";
		controller.uno_serial->write(ss.str());
		ros::spinOnce(); //basically calls the callback function: updatse the velocities 
		loop_rate.sleep();
	}
}
