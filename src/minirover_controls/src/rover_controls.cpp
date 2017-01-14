#include <iostream>
#include <string>

#include "drivetrain_control.h"
#include "serial/serial.h"

using std::string;
using std::cout; using std::cin; using std::endl;

string uno_port_c = "/dev/ttyACM0";
int uno_baud_c 	= 9600;
//string temp = "9600";

serial::Serial * connect_to_serial(string, int);

int 
main(int argc, char** argv){
	ros::init(argc, argv, "rover_actuator_control_node"); //initializes the node 
	ros::NodeHandle nh;
	nh.param("serial_port", uno_port_c, uno_port_c);
	nh.param("baud_rate", uno_baud_c, uno_baud_c);

	serial::Serial * uno_serial = connect_to_serial(uno_port_c, uno_baud_c);
	DriveTrainControl drivetrain_controller(uno_serial);
	stringstream ss; 
	while(ros::ok()) {
		drivetrain_controller.transmit();
		ros::spinOnce(); //basically calls the callback function: updatse the velocities 
		ros::Duration(.1).sleep();
	}
}

serial::Serial*
connect_to_serial(string port, int baud){
	bool connected(false);
	serial::Serial * serial;
	while(!connected){
		try {
			serial = new serial::Serial(port, baud, 
						serial::Timeout::simpleTimeout(1000));
			connected = true;
		} catch(serial::IOException e){
			std::cout << "Error connecting to serial port " << port << std::endl;
			cout      << "Enter a different serial port or kill the program [ Default: " << port << " ]" << endl;
    			string input;
    			getline( cin, input );
    			if ( !input.empty() ) port = input;
		}
	}
	return serial;
}
