#include <iostream>
#include <string>

#include "actuator_control.h"
#include "motor_control.h "
#include "serial/serial.h"

using std::string;
using std::cout; using std::cin; using std::endl;

const string uno_port_c = "/dev/ttyACM0";
const int uno_baud_c 	= 9600;

serial::Serial * connect_to_serial(string, int);

int 
main(int argc, char** argv){
	ros::init(argc, argv, "rover_actuator_control_node"); //initializes the node 

	serial::Serial * uno_serial = connect_to_serial(uno_port_c, uno_baud_c);

	ExcavatorControl excavator_controller(uno_serial);
	WheelControl 	 wheel_controller(uno_serial);
	
	ros::Rate loop_rate(10000); //only run while loop every 10 milliseconds 
	while(ros::ok()) {	
		excavator_controller.Transmit();
		wheel_controller.Transmit();
		ros::spinOnce(); //basically calls the callback function: updatse the velocities 
		loop_rate.sleep();
	}
}

serial::Serial*
connect_to_serial(string port, int baud){
	bool connected(false);
	while(!connected)
		try {
			uno_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
			connected = true;
		} catch(serial::IOException e){
			std::cout << "Error connecting to serial port " << port << std::endl;
			cout << "Enter a different serial port or kill the program [ Default: " << port " ]" << endl;
    		string input;
    		getline( cin, input );
    		if ( !input.empty() ) port = input;
		}
	}
}