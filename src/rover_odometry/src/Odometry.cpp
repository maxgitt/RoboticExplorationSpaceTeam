#include <Odometry/Odometry.h>
//#include <rover_odometry/Odometry.h> //message class
#include <stdio.h>
#include <set>
#include <iostream>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <time.h> //needed to run a while loop for a fixed amount of time
#include <unordered_set>

using std::cin; using std::cout; using std::endl; using std::cerr;
using std::stringstream; using std::istringstream; using std::string;
using std::unordered_map; using std::unordered_set;
using std::pair;
using std::deque;
using std::numeric_limits;
using std::vector;
using std::string;

Odometry::Odometry(){
	string sieve_ids;
	string id; //individual sieve beacon
	string offsetValues ="-100,-100"; //x,y offset of the sieve beacon
	string beacon_ids = "48c36259,35a79eb,997ca3f0,c3e3c227";
	vector< std::pair<int, RoverBeacon::sieveBeaconData> > sieveBeacons;

	//Get list of sieve beacons addresses from parameter server
	nh.param(param_key + "sieve_beacon_ids", sieve_ids, sieve_ids);
	//parse each address and check parameter server for offset.
	// parameter server must be loaded
	//with xy offsets for each address of form x + <id>
	cout << "Read the following sieve_ids from the parameter server: " <<
							 sieve_ids << endl;
	istringstream sieve_beacon_ids(sieve_ids);
	while(getline(sieve_beacon_ids, id, ',')){
		string offsetparam = "x" + id;
		nh.param(param_key + offsetparam, offsetValues, offsetValues);
		cout << "The offset for sieve_id: " << id << 
					" is " << offsetValues << endl;
		istringstream offset(offsetValues);
		string x = "-1";
		string y = "-1";
		getline(offset, x, ',');
		getline(offset, y);

		pair<double,double> temp(stod(x),stod(y));
		RoverBeacon::sieveBeaconData newSieveBeaconData(temp);
		pair<int, RoverBeacon::sieveBeaconData> beaconToBeAdded(stoi(id),
							newSieveBeaconData);
		sieveBeacons.push_back(beaconToBeAdded);
	}
	numSieveBeacons = sieveBeacons.size();


	//Initialize rover beacons	
	nh.param(param_key + "rover_beacon_ids", beacon_ids, beacon_ids);
	cout << "Loading the following rover beacon ids from the parameter server: " << beacon_ids << endl;
	istringstream rover_beacon_ids(beacon_ids);
	while (getline(rover_beacon_ids, id, ',')) {
		//this is how rover offset is stored in the parameter server
		string offsetparam = param_key + "r" + id;
		nh.param(offsetparam, offsetValues, offsetValues);
		cout << "Rover beacon: " << id << " has offsets " <<
							 offsetValues;
		istringstream offset(offsetValues);
		string x = "-1";
		string y = "-1";
		getline(offset, x, ',');
		getline(offset, y);
		pair<double,double> roverOffset(stod(x),stod(y));
		RoverBeacon temp(id, sieveBeacons, roverOffset);
		RoverBeacons.push_back(temp);
	}
	for(auto it = RoverBeacons.begin(); 
					it != RoverBeacons.end(); ++it){
		//cout << &(*it) << endl;
		RoverBeaconsMap[(*it).getId()]= &(*it);
	}
	//initialize odom message with 50 message queue
	odom_pub     = nh.advertise<nav_msgs::Odometry>("odom", 50);
	//odom_broadcaster = _odom_broadcaster;
	current_time = ros::Time::now();
	last_time    = ros::Time::now();
}

// comes in as
// roverid, sieveId, distance
void
Odometry::detectRoverBeacons(){
	int numRoverBeacons = RoverBeacons.size();
	unordered_set<string> uniqueRoverBeacons;
	cout << "Detecting the beacons that are connected to the rover\n" 
		<< "We expect " << RoverBeacons.size() << " Beacons!" << endl;
	string data;
	string roverBeaconID;
	stringstream fileData;
	//keep looking for the expected number of rover beacons until 
	//timeout at which point throw an exception
	getline(cin,data);
	while(getline(cin, data)){
		//cout << "Current line of data being read is " << data << endl;
		fileData.str(data);
		getline(fileData, roverBeaconID, ',');
		uniqueRoverBeacons.insert(roverBeaconID);
		if (uniqueRoverBeacons.size() == numRoverBeacons) {
			cout << "Identified all the beacons!" << endl;
 			return;
		}
	}
}

void 
Odometry::loadBiases(){
	std::string defaultValue = "ERROR";
	// In server, it should be designated "roverId sieveId" with value bias
	//cout << "Just reaching loadBiases()" << endl;
	for (auto rbeaconIt = RoverBeacons.begin(); 
			rbeaconIt != RoverBeacons.end(); ++rbeaconIt) {
		unordered_map<int, RoverBeacon::sieveBeaconData> 				currBeaconReadings = rbeaconIt->getBeaconReadings();
		for (auto it = currBeaconReadings.begin();
			it != currBeaconReadings.end(); ++it) {
			string currBias = "-1";
			string currBiasPair = rbeaconIt->getId() + 
					"_" + std::to_string(it->first);


			//nh.param(param_key + currBiasPair, currBias);

			if      (currBiasPair == "48c36259_24")
				currBias = "4.41";  //tested to be between 2.8-3.3
			else if (currBiasPair == "48c36259_13")
				currBias = "3.83"; //tested to be between 2.2 and 2.8
			else if (currBiasPair == "c3e3c227_24")
				currBias = "7.33"; //tested to be between 4.9 and 5.57
			else if (currBiasPair == "c3e3c227_13")
				currBias = "6.8"; //ested to be between 4.8 and 5.4
			else if (currBiasPair == "c3e3c227_23") 
				currBias = "-5.48";
			else if (currBiasPair == "48c36259_23")
				currBias = "-8.55";
			rbeaconIt->updateBias(it->first, stod(currBias));
		}
	}	
}


void 
Odometry::updateOdometry(){


	cout << "Reached update odometry!" << endl;
	updateBeaconReadings();

	if(first){
		current_time = ros::Time::now();
		last_time = ros::Time::now();
	}

	//update the previous values before calculating the new values
	x_prev = x;
	y_prev = y;
	th_prev = th;

	double x_val = 0;
	double y_val = 0;
	//take position readings from each rover beacon and then average them
	for( auto it = RoverBeacons.begin(); it != RoverBeacons.end(); ++it) {
		cout << "Adding a x yalue of " << 
			((*it).getPosition()).first + (*it).getOffset().first;
		x_val += ((*it).getPosition()).first + (*it).getOffset().first;
		
		cout << "Adding a y value of " << ((*it).getPosition()).second +
					 (*it).getOffset().second << std::endl;
		
		y_val += ((*it).getPosition()).second + (*it).getOffset().second;
	}
	x = x_val/RoverBeacons.size();
	y = y_val/RoverBeacons.size();

	//calculate the new theta
	double theta = 0;
	vector<RoverBeacon>::iterator firstBeacon = RoverBeacons.begin();
	vector<RoverBeacon>::iterator secondBeacon = ++RoverBeacons.begin();
	while (secondBeacon != RoverBeacons.end()) {
		while (secondBeacon != RoverBeacons.end()) {
    			double theta_temp = 0;
			theta_temp += calcAngle(firstBeacon->getPosition(),
					 secondBeacon->getPosition());
			theta_temp -= calcAngle(firstBeacon->getOffset(),
					 secondBeacon->getOffset());
			theta_temp += 90;
			cout << "theta temp is : " << theta_temp << endl;
			theta += theta_temp;
			++secondBeacon;
		}
		++firstBeacon;
		secondBeacon = firstBeacon;
		++secondBeacon;
	}
	//change when we add more rover beacons
	theta /= 1;
	std::cout << "x position: " << x << std::endl << "y position: " << y << std::endl << "Angle: " << theta << std::endl; 

	if(first){
		// if it is the first message we are just initializing
		// we cannot calculate valid odometry with only one point
		first = false;
		return;
	}

	publish_odometry();
}

void
Odometry::publish_odometry(){
    	current_time = ros::Time::now();

    	//compute odometry in a typical way given the velocities of the robot
	dt = (current_time - last_time).toSec();
    
	vx  = (x  - x_prev)  / dt;
	vy  = (y  - x_prev)  / dt;
	vth = (th - th_prev) / dt;

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vth;

	//publish the message
	odom_pub.publish(odom);

	last_time = current_time;
}


//Always pass in the higher y-value as firstPos!!!!!!!
double Odometry::calcAngle(pair<double,double> firstPos, pair<double,double> secondPos) {
	double x_val = firstPos.first - secondPos.first;
	double y_val = firstPos.second - secondPos.second;
	double return_angle = atan2(y_val,x_val) * (180/3.14159);
	cout << "RETURNING: " << return_angle << endl;
	return return_angle;
}

//will store the values as a pair of id,reading for each beacon and update the readings. We will need to add a timeout!!
void Odometry::updateBeaconReadings() {
	cout << "Going to update the Beacon Readings\n";
	unordered_set<std::string> currentReadings;
	string beaconID, sieveID, distance_;
	double distance = 5;
	string line;
	getline(cin,line);//std::getline(driverData.out(),line);

	while (currentReadings.size() < (numSieveBeacons * RoverBeacons.size())) {
		stringstream inputStream;
		string line;
		getline(cin,line);
		inputStream.str(line);
		getline(inputStream, beaconID, ',');
		getline(inputStream, sieveID, ',');
		getline(inputStream, distance_);

		try {
			distance = stod(distance_);
			RoverBeacon* temp = RoverBeaconsMap[beaconID];
			assert(temp);
			RoverBeaconsMap[beaconID]->updateReading(stoi(sieveID),
								 distance);
			currentReadings.insert(beaconID + sieveID);

		}
		catch (std::invalid_argument error) {
			cerr<< "Invalid Argument: " << error.what() << "\n";
		}
	}
}

pair<double,double> 
Odometry::getPosition(){
    return roverPosition;
}

double 
Odometry::getPose(){
    return roverPose;
}


