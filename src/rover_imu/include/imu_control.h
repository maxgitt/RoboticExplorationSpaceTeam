#include <iostream>
#include <cmath>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"

using namespace std;

class IMUControl {

public:
	IMUControl();
	void receive();
private:

	int right_vel = 0;
	int left_vel = 0;

	ros::NodeHandle nh;

	// Robot Dimensions
	double robot_width = 0;
	double wheel_diameter = 0;
	double max_speed = 0;

	// Serial Connection Variables
	string serial_port = "";
	int serial_baud = 0; 
	serial::Serial * serial;

	// Network Configuration
	string imu_topic = "imu_data";

};

//Initialize serial port 
IMUControl::IMUControl() {
	nh.param("/rover_imu/serial_port", serial_port, serial_port);
	nh.param("/rover_imu/serial_baud", serial_baud, serial_baud);
	nh.param("/rover_imu/imu_topic", imu_topic, imu_topic);


	try {
		serial = new serial::Serial(serial_port, serial_baud, serial::Timeout::simpleTimeout(1000));
	} catch(serial::IOException e){
		cerr << "Error connecting to serial in IMUController" << endl;
		exit(0);
	}

}

void 
IMUControl::receive(){
	string reading;
	if(serial->available()) {
		size_t ret = serial->readline(reading);
	}
	cerr << reading << endl;
}
