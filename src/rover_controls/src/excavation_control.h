#include <iostream>
#include <cmath>
#include <sstream>

#include "ros/ros.h"
#include "serial/serial.h"
#include "rover_controls/Excavation.h"


using namespace std;

class ExcavatorControl {

public:
	ExcavatorControl() = delete;
	ExcavatorControl(serial::Serial *);
	serial::Serial * uno_serial;
	void transmit();
private:

	void callback(const rover_controls::Excavation &);

	int left_actuator		  = 0;
	int right_actuator		  = 0;
	double excavator_speed 		  = 90;

	ros::NodeHandle nh;
	ros::Subscriber cmd_excv_sub; 
};

//Initialize serial port 
ExcavatorControl::ExcavatorControl(serial::Serial * _uno_serial)  {
	uno_serial = _uno_serial;
	cmd_excv_sub = nh.subscribe("/cmd_excv", 10, &ExcavatorControl::callback, this);
}

void
ExcavatorControl::callback(const rover_controls::Excavation &excv_msg){
	left_actuator  = excv_msg.left_actuator;
	right_actuator = excv_msg.right_actuator;
	excavator_speed = excv_msg.excavator_speed * 90 + 90;
}

void
ExcavatorControl::transmit() {
	stringstream ss; 
	ss << '1' << ',' << left_actuator << "," << right_actuator << "," << excavator_speed << "\n";
	uno_serial->write(ss.str());
}
