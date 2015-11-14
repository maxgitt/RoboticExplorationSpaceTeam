#include <ros/ros.h>
#include "serial/serial.h"
#include <rover_motor_control/Encoder.h>
#include <iostream>
#include <sstream>
#include <string>
#include <cstring>

class EncoderControl
{
public:
  EncoderControl();
  void update_encoder();

private:
	int serial_encoder_BAUD_;
	std::string serial_encoder_PORT_;
	serial::Serial * uno_serial;
  ros::NodeHandle nh_;
  ros::Publisher encoder_pub_;
};

EncoderControl::EncoderControl():
	serial_encoder_BAUD_(9600),
	serial_encoder_PORT_("ttyACM0")
{
	nh_.param("serial_encoder_BAUD", serial_encoder_BAUD_, serial_encoder_BAUD_);
	nh_.param("serial_encoder_PORT", serial_encoder_PORT_, serial_encoder_PORT_);

	try{
		uno_serial = new serial::Serial(serial_encoder_PORT_, serial_encoder_BAUD_, serial::Timeout::simpleTimeout(1000));	
	}
	catch(serial::IOException e){
		std::cerr << "Could not connect\n";
	}

  encoder_pub_ = nh_.advertise<rover_motor_control::Encoder>("encoder_values", 10);
}

void EncoderControl::update_encoder()
{
	std::string msg_in = "45,60,3,4";//uno_serial.read();

	std::stringstream ss(msg_in);
	
	rover_motor_control::Encoder msg;

	int x;
	char delim;

	ss >> x;
	msg.back_left_ = x;

	ss >> delim;
	ss >> x;
	msg.back_right_ = x;

	ss >> delim;
	ss >> x;
	msg.front_left_ = x;

	ss >> delim;
	ss >> x;
	msg.front_right_ = x;

	/*getline(ss, line, ',');
	msg.back_left_ = std::stoi(line);

	getline(ss, line, ',');
	msg.back_right_ = std::stoi(line);

	getline(ss, line, ',');
	msg.front_left_ = std::stoi(line);

	getline(ss, line);
	msg.front_right_ = std::stoi(line);
	*/
	encoder_pub_.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "encoder_control_REST");
  EncoderControl encoder_control;
	ros::Rate loop_rate(10);
  while (ros::ok()){
		encoder_control.update_encoder();
  	ros::spinOnce();
		loop_rate.sleep();
	}
}
