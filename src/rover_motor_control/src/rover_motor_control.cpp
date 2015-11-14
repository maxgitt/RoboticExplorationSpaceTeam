#include "ros/ros.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

//serial setup
#define BAUD 9600
#define PORT "ttyACM0"

//rover physical properties
#define ROBOT_WIDTH 1 //meters
#define WHEEL_DIAMETER .5 //meters

using namespace std;

class RoverMotorControl {

public:
	RoverMotorControl();
	double getvl();
	double getvr();

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
	geometry_msgs::Twist twist = twist_aux;
	double vel_x = twist_aux.linear.x;
	double vel_th = twist_aux.angular.z;
	double right_vel = 0.0;
	double left_vel = 0.0;
	if(vel_x == 0){
		// turning
		right_vel = vel_th * ROBOT_WIDTH / 2.0;
		left_vel = (-1) * right_vel;
	}else if(vel_th == 0){
		// forward / backward
		left_vel = right_vel = vel_x;
	}else{
		// moving doing arcs
		left_vel = vel_x - vel_th * ROBOT_WIDTH / 2.0;
		right_vel = vel_x + vel_th * ROBOT_WIDTH / 2.0;
	}
	vl_ = left_vel;
	vr_ = right_vel;
}

RoverMotorControl::RoverMotorControl()
{
	vl_ = 90;
	vr_ = 90;

	ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, &RoverMotorControl::callback, this);
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
	ros::init(argc, argv, "motor_control_rest");
	RoverMotorControl controller;
	bool good = false;
	while(!good){
		try {
			serial::Serial uno_serial(PORT, BAUD, serial::Timeout::simpleTimeout(1000));
		}
		catch(serial::IOException e){
			std::cerr << "ERROR connecting to serial" << std::endl;
			good = false;
		}
	}
	ros::Rate loop_rate(10);
	while(ros::ok())
	{	
		stringstream ss;
		ss << controller.getvl() << "," << controller.getvr() << "," << controller.getvl() << "," << controller.getvr() << "\n";
		//uno_serial.write(ss.str());
		ros::spinOnce();
		loop_rate.sleep();
	}
}
