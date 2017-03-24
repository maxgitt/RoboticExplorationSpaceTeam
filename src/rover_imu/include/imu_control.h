#include <iostream>
#include <cmath>
#include <sstream>
#include <string>
#include <stdint.h>
#include <stdio.h>
#include <climits>
#include <math.h>
#include "ros/ros.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"

using namespace std;

class IMUControl {

public:
	IMUControl();
	void receive();
	void publish_raw();

	float get_yaw();
	float get_pitch();
	float get_roll();

	float get_avx();
	float get_avy();
	float get_avz();

	float get_lax();
	float get_lay();
	float get_laz();	

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
	string imu_topic = "imu_data_raw";
	ros::Publisher imu_pub;


	float quaternion[4]; // yaw pitch roll
	float gyro[3];
	float acc[3]; // accx accy accz angx angy angz
	float acc_bias[3] = {0,0,0};
	int acc_bc = 0;
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
	imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 1);
}

void 
IMUControl::receive(){
	if(serial->available()) {
		string tmp;
		string line = serial->readline();
		stringstream ss(line);

		uint32_t num;
		for(int i = 0; i < 3; ++i) {
			ss >> tmp;
			sscanf(tmp.c_str(), "%x", &num);  // assuming you checked input
			num = ((num>>24)&0xff) | // move byte 3 to byte 0
                    ((num<<8)&0xff0000) | // move byte 1 to byte 2
                    ((num>>8)&0xff00) | // move byte 2 to byte 1
                    ((num<<24)&0xff000000); // byte 0 to byte 3
			quaternion[i] = *((float*)&num);
		}

		for(int i = 0; i < 3; ++i) {
			ss >> tmp;
			sscanf(tmp.c_str(), "%x", &num);  // assuming you checked input
			num = ((num>>24)&0xff) | // move byte 3 to byte 0
                    ((num<<8)&0xff0000) | // move byte 1 to byte 2
                    ((num>>8)&0xff00) | // move byte 2 to byte 1
                    ((num<<24)&0xff000000); // byte 0 to byte 3
			gyro[i] = *((float*)&num);
		}


		for(int i = 0; i < 3; ++i) {
			ss >> tmp;
			sscanf(tmp.c_str(), "%x", &num);  // assuming you checked input
			num = ((num>>24)&0xff) | // move byte 3 to byte 0
                    ((num<<8)&0xff0000) | // move byte 1 to byte 2
                    ((num>>8)&0xff00) | // move byte 2 to byte 1
                    ((num<<24)&0xff000000); // byte 0 to byte 3
			acc[i] = *((float*)&num);
		}


		if(acc_bc == 20) {
			cerr << "in herrr" << endl;
			memcpy(&acc_bias, &acc, 3*sizeof(float));
		}
		++acc_bc;

		publish_raw();
	}
}

void
IMUControl::publish_raw(){
	sensor_msgs::Imu imu_msg = sensor_msgs::Imu();	
	imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu";
    imu_msg.orientation.x = get_roll();
    imu_msg.orientation.y = get_pitch();
    imu_msg.orientation.z = get_yaw();
    imu_msg.orientation.w = 0.0;
    //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    imu_msg.angular_velocity.x = get_avx();
    imu_msg.angular_velocity.y = get_avy();
    imu_msg.angular_velocity.z = get_avz();
    //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    imu_msg.linear_acceleration.x = get_lax();
    imu_msg.linear_acceleration.y = get_lay();
    imu_msg.linear_acceleration.z = get_laz();
    //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};	
	//publish the new imu message
	imu_pub.publish(imu_msg);
}


float
IMUControl::get_yaw() {
	return quaternion[0];
}

float
IMUControl::get_pitch() {
	return quaternion[1];
}

float
IMUControl::get_roll() {
	return quaternion[2];
}

float
IMUControl::get_lax() {
	return (acc[0] - acc_bias[0]) * 9.80665;
}
float
IMUControl::get_lay() {
	return (acc[1] - acc_bias[1]) * 9.80665;
}
float
IMUControl::get_laz() {
	return (acc[2] - acc_bias[2]) * 9.80665;
}

float
IMUControl::get_avx() {
	return gyro[0] * M_PI / 180;
}
float
IMUControl::get_avy() {
	return gyro[1] * M_PI / 180;
}
float
IMUControl::get_avz() {
	return gyro[2] * M_PI / 180;
}

