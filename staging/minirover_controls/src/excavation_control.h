#include <iostream>
#include <cmath>
#include <sstream>

#include "ros/ros.h"
#include "serial/serial.h"
#include "minirover_controls/MExcavation.h"

using namespace std;

class ExcavatorControl {

public:
	ExcavatorControl() = delete;
	ExcavatorControl(serial::Serial *);
	serial::Serial * uno_serial;
	void transmit();
private:

	void callback(const rover_controls::Excavation &);
	
	bool is_manual 	  		  = true;

	int left_actuator		  = 0;
	int right_actuator		  = 0;

	double actuator_percent_extended  = 0;
	bool read_sensor 		  = false;
	double excavator_speed 		  = 90;

	ros::NodeHandle nh;
	ros::Subscriber cmd_excv_sub; 
	ros::Publisher capacity_pub; 
};

//Initialize serial port 
ExcavatorControl::ExcavatorControl(serial::Serial * _uno_serial)  {
	uno_serial   = _uno_serial;
	cmd_excv_sub = nh.subscribe("/cmd_excv", 10, &ExcavatorControl::callback, this);
  	capacity_pub      = nh.advertise<rover_controls::Capacity>("capacity", 1);
}

void
ExcavatorControl::callback(const rover_controls::Excavation &excv_msg){
	if(excv_msg.manual) {

		is_manual = true;
		left_actuator  = excv_msg.left_actuator;
		right_actuator = excv_msg.right_actuator;
		excavator_speed = excv_msg.excavator_speed * 90 + 90;
	}
	else {
		is_manual = false;
		actuator_percent_extended = excv_msg.actuator_percent_extended;
		excavator_speed = excv_msg.excavator_speed * 90 + 90;
	}
	
	if(excv_msg.update_capacity) {
		read_sensor = true;
	}

}

bool is_number(const std::string& s)
{
    return !s.empty() && std::find_if(s.begin(), 
        s.end(), [](char c) { return !std::isdigit(c); }) == s.end();
}

void
ExcavatorControl::transmit() {
	stringstream ss; 
	if(is_manual) {
		ss << '1' << ',' << left_actuator << "," << right_actuator << "," << excavator_speed << "\n";
	}
	else if ( !is_manual) {
		ss << '2' << ',' << left_actuator << "," << right_actuator << "," << excavator_speed << "\n";
	}
	uno_serial->write(ss.str());
	
	if(read_sensor){
		read_sensor = false;
		uno_serial->write("3\n'");
		string reading = uno_serial->readline();
		rover_controls::Capacity cap;
		cap.reading = atoi(reading.c_str());
		capacity_pub.publish(cap);
	}
}
