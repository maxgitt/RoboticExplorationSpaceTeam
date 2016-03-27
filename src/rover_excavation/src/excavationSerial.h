#include <ros/ros.h>
#include "serial/serial.h"
#include <iostream>
#include <sstream>
#include <string>

class SerialComm
{
public:
SerialComm();
	serial::Serial * uno_serial;
	bool commandLinearActuator(std::string command);

private:
	int linear_actuator_BAUD_;
	std::string linear_actuator_PORT_;
  ros::NodeHandle nh_;
};

SerialComm::SerialComm():
	linear_actuator_BAUD_(9600),
	linear_actuator_PORT_("ttyACM0")
{
	nh_.param("linear_actuator_BAUD", linear_actuator_BAUD_, linear_actuator_BAUD_);
	nh_.param("linear_actuator_PORT", linear_actuator_PORT_, linear_actuator_PORT_);

	try{
		uno_serial = new serial::Serial(linear_actuator_PORT_, linear_actuator_BAUD_, serial::Timeout::simpleTimeout(1000));	
	}
	catch(serial::IOException e){
		std::cerr << "Could not connect\n";
	}
}

bool SerialComm::commandLinearActuator(std::string command)
{
	std::string msg_in = uno_serial->read();
	std::stringstream ss(msg_in);	
	std::string msg_out;
	
	
	if(command == "extend")
	{
		uno_serial->write("extend");
	}
	else if(command == "retract")
	{
		uno_serial->write("retract");
	}
	else
	{
		std::cerr << "Message not recognized\n";
		return false;
	}
	return true;

}

