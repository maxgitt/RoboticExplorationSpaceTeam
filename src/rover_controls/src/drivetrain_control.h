#include <iostream>
#include <cmath>
#include <sstream>
#include "ros/ros.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"

//rover physical properties
#define ROBOT_WIDTH 1 //meters
#define WHEEL_DIAMETER .3 //meters

using namespace std;

class DriveTrainControl {

public:
	DriveTrainControl() = delete;
	DriveTrainControl(serial::Serial * _uno_serial);
	void transmit();
private:
	void callback(const geometry_msgs::Twist &twist_aux);

	double vl = 90;
	double vr = 90;
	double robot_width, wheel_diameter;

	ros::NodeHandle nh;
	ros::Subscriber cmd_vel_sub; 
	serial::Serial * uno_serial;
};

//Initialize serial port 
DriveTrainControl::DriveTrainControl(serial::Serial * _uno_serial) {
	uno_serial = _uno_serial;
	cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &DriveTrainControl::callback, this);
}

void
DriveTrainControl::callback(const geometry_msgs::Twist &twist_aux){
	//cout << twist_aux.linear.x << twist_aux.angular.z << endl;
	double vel_x =  twist_aux.linear.x;
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
	vl =  -1 * (left_vel - 180) ;
	vr =  -1 * (right_vel - 180);
}

void 
DriveTrainControl::transmit(){
	stringstream ss; 
	ss << '0' << ',' << vl << ',' << vr << ',' << vl << ',' << vr << '\n';		
	uno_serial->write(ss.str());
}
