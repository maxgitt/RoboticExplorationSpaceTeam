#include <iostream>
#include <cmath>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"

using namespace std;

class DriveTrainControl {

public:
	DriveTrainControl();
	void transmit();
	void receive();
private:
	void callback(const geometry_msgs::Twist &twist_aux);

	int right_vel = 0;
	int left_vel = 0;

	ros::NodeHandle nh;
	ros::Subscriber cmd_vel_sub;

	// Robot Dimensions
	double robot_width = 0;
	double wheel_diameter = 0;
	double max_speed = 0;

	// Serial Connection Variables
	string serial_port = "";
	int serial_baud = 0; 
	serial::Serial * serial;

	// Network Configuration
	string command_topic = "cmd_vel";

	// Speed updated?
	bool speed_updated = false;
	float wheel_speeds[2];
};

//Initialize serial port 
DriveTrainControl::DriveTrainControl() {
	nh.param("/rover_controls/drivetrain/serial_port", serial_port, serial_port);
	nh.param("/rover_controls/drivetrain/serial_baud", serial_baud, serial_baud);
	nh.param("/rover_controls/drivetrain/command_topic", command_topic, command_topic);
	nh.param("/rover_description/robot_width", robot_width, robot_width);
	nh.param("/rover_description/wheel_diameter", wheel_diameter, wheel_diameter);
	nh.param("/rover_description/max_speed", max_speed, max_speed);

	try {
		serial = new serial::Serial(serial_port, serial_baud, serial::Timeout::simpleTimeout(1000));
	} catch(serial::IOException e){
		cerr << "Error connecting to serial in DriveTrainCotroller" << endl;
		exit(0);
	}

	cmd_vel_sub = nh.subscribe(command_topic, 10, &DriveTrainControl::callback, this);
}

void
DriveTrainControl::callback(const geometry_msgs::Twist &twist_aux){
  double vel_x  = twist_aux.linear.x;
  double vel_th = twist_aux.angular.z;
  if(abs(vel_x) == 0){
    // turning
    right_vel = vel_th * robot_width / 2.0 * (max_speed * 255);
    left_vel = -1 * right_vel;
  } else if(abs(vel_th) == 0){
    // forward / backward
    left_vel  = vel_x * (max_speed * 255);
    right_vel = vel_x * (max_speed * 255);
  } else{
    // moving doing arcs
    left_vel = (vel_x - vel_th * robot_width / 2.0) * (max_speed * 255);
    right_vel = (vel_x + vel_th * robot_width / 2.0) * (max_speed * 255);
  }	
  std::swap(left_vel, right_vel);
  speed_updated = true;	
}

void 
DriveTrainControl::transmit(){
	if(!speed_updated) {
		return;
	}
	else {
		speed_updated = false;
	}
	stringstream ss; 
	ss << '1' << ' ' << left_vel <<  ' ' << right_vel << '\n';		
	serial->write(ss.str());	
}


void 
DriveTrainControl::receive(){
	string line;
	string tmp;
	if(serial->available()) {
		size_t ret = serial->readline(line);
		stringstream ss(line);
	
		uint32_t num;
		for(int i = 0; i < 2; ++i) {
			ss >> tmp;
			sscanf(tmp.c_str(), "%x", &num);  // assuming you checked input
			num = ((num>>24)&0xff) | // move byte 3 to byte 0
	                ((num<<8)&0xff0000) | // move byte 1 to byte 2
	                ((num>>8)&0xff00) | // move byte 2 to byte 1
	                ((num<<24)&0xff000000); // byte 0 to byte 3
			wheel_speeds[i] = *((float*)&num);
		}
		for(auto i : wheel_speeds)
			cerr << i << ' ';
		cerr << endl;
	}
}
