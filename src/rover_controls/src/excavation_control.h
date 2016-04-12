#include "ros/ros.h"
#include "serial/serial.h"
#include "excavation_control/Excavation.h"
#include <iostream>
#include <cmath>

using namespace std;

struct 
ActuatorSignal {
	int wire_one = 0;
	int wire_two = 0;
};

class ExcavationControl {

public:
	ExcavationControl() = delete;
	ExcavationControl(serial::Serial *);
	serial::Serial * uno_serial;
private:
	void transmit();
	void callback(const rover_actuator_control::Actuator &);

	ActuatorSignal left_actuator  = {0,0};
	ActuatorSignal right_actuator = {0,0};
	double excavator_speed 		  = 90;

	ros::NodeHandle nh;
	ros::Subscriber cmd_excv_sub; 
};

//Initialize serial port 
ExcavationControl::ExcavationControl(serial::Serial * _uno_serial)  {
	uno_serial = _uno_serial;
	cmd_excv_sub = nh.subscribe("/cmd_excv", 10, &ExcavationControl::callback, this);
}

void
ExcavationControl::callback(const excavation_control::Excavation &excv_msg){
	int templ  = excv_msg.left_actuator;
	int tempr  = excv_msg.right_actutor;

	if( templ == 0 ){
		left_actuator.wire_one = 0;
		left_actuator.wire_two = 0;
	} else if ( templ == 1 ) {
		left_actuator.wire_one = 0;
		left_actuator.wire_two = 1;
	} else if ( templ == -1 ) {
		left_actuator.wire_one = 1;
		left_actuator.wire_two = 0;
	} 

	if( tempr == 0 ){
		right_actuator.wire_one = 0;
		right_actuator.wire_two = 0;
	} else if ( tempr == 1 ) {
		right_actuator.wire_one = 0;
		right_actuator.wire_two = 1;
	} else if ( tempr == -1 ) {
		right_actuator.wire_one = 1;
		right_actuator.wire_two = 0;
	} 

	excavator_speed = excv_msg.excavator_speed * 90 + 90;
}

void
ExcavationControl::Transmit() {
	stringstream ss; 
	ss << left_actuator << "," << right_actuator << "," << excavator_speed << "\n";
	controller.uno_serial->write(ss.str());
}
