#include <Odometry/Odometry.h>
#include <sstream>
#include <string>
#include <stdio.h>
#include <set>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <unordered_set>
#include <cassert>
#include <time.h> //needed to run a while loop for a fixed amount of time

Odometry::Odometry(){
	string sieve_ids;
	string id; //individual sieve beacon
	string offsetValues ="-100,-100"; //x,y offset of the sieve beacon
	string beacon_ids = "48c36259,35a79eb,997ca3f0,c3e3c227";
	vector< std::pair<int, RoverBeacon::sieveBeaconData> > sieveBeacons;

	//Get list of sieve beacons addresses from parameter server
		nh.param(param_key + "sieve_beacon_ids", sieve_ids, sieve_ids);
	//parse each address and check parameter server for offset. parameter server must be loaded
	//with xy offsets for each address of form x + <id>
	istringstream sieve_beacon_ids(sieve_ids);
	while(getline(sieve_beacon_ids, id, ',')){
		std::string offsetparam = "x" + id;
		nh.param(param_key + offsetparam, offsetValues, offsetValues);
		istringstream offset(offsetValues);
		string x = "-1";
		string y = "-1";
		getline(offset, x, ',');
		getline(offset, y);

		pair<double,double> temp(stod(x),stod(y));
		RoverBeacon::sieveBeaconData newSieveBeaconData(temp);
		pair<int, RoverBeacon::sieveBeaconData> beaconToBeAdded(stoi(id),newSieveBeaconData);
		sieveBeacons.push_back(beaconToBeAdded);
		numSieveBeacons = sieveBeacons.size();
	}

	//Initialize rover beacons	
	nh.param(param_key + "rover_beacon_ids", beacon_ids, beacon_ids);
	istringstream rover_beacon_ids(beacon_ids);
	while (getline(rover_beacon_ids, id, ',')) {
		//this is how rover offset is stored in the parameter server
		string offsetparam = param_key + "r" + id;
		nh.param(offsetparam, offsetValues, offsetValues);
		istringstream offset(offsetValues);
		string x = "-1";
		string y = "-1";
		getline(offset, x, ',');
		getline(offset, y);
		cout << id << endl;
		pair<double,double> roverOffset(stod(x),stod(y));
		RoverBeacon temp(id, sieveBeacons, roverOffset);
		cout << &temp << endl;
		RoverBeacons.push_back(temp);
	}
	cout << &RoverBeacons.at(0) << endl;
	cout << &RoverBeacons.at(1) << endl;
	for(vector<RoverBeacon>::iterator it = RoverBeacons.begin(); it != RoverBeacons.end(); ++it){
		cout << &(*it) << endl;
		RoverBeaconsMap[(*it).getId()]= &(*it);
	}
	cout << RoverBeaconsMap["c3e3c227"] << endl;
	cout << RoverBeaconsMap["48c36259"] << endl; 
  
	//initialize odom message with 50 message queue
	odom_pub     = nh.advertise<nav_msgs::Odometry>("odom", 50);
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
	//keep looking for the expected number of rover beacons until timeout at which point throw an exception
	getline(cin,data);
		if(data == "") {
			cerr << "Consuming useless line failed!" << endl;
		}
  
	while(getline(cin, data,',')){
		fileData.str(data);
		getline(fileData, roverBeaconID, ',');
		uniqueRoverBeacons.insert(roverBeaconID);
		if (uniqueRoverBeacons.size() == numRoverBeacons) return;
	}
}

void 
Odometry::loadBiases(){
	std::string defaultValue = "ERROR";
	    // In server, it should be designated "roverId sieveId" with value bias
	for (auto rbeacon: RoverBeacons) {
		unordered_map<int, RoverBeacon::sieveBeaconData> currBeaconReadings = rbeacon.getBeaconReadings();
		for (unordered_map<int, RoverBeacon::sieveBeaconData>::iterator it = currBeaconReadings.begin();
									 it != currBeaconReadings.end(); ++it) {
			string currBias = "-1";
			string currBiasPair = rbeacon.getId() + "_" + std::to_string(it->first);
			nh.param(param_key + currBiasPair, currBias, defaultValue);
			rbeacon.updateBias(it->first, stod(currBias));
		}
	}	
cout << "Reached end of load biases" << endl;
}


void 
Odometry::updateOdometry(){
	cout << "Reached update odometry!" << endl;
	updateBeaconReadings();
	//update the previous values before calculating the new values
	x_prev = x;
	y_prev = y;
	th_prev = th;

	double x_val = 0;
	double y_val = 0;
	//take position readings from each rover beacon and then average them
	for (vector<RoverBeacon>::iterator it = RoverBeacons.begin(); it != RoverBeacons.end(); ++it) {
		x_val += ((*it).getPosition()).first + (*it).getOffset().first;
		//std::cout << "Adding a y value of " << ((*it).getPosition()).second + (*it).getOffset().second << std::endl;
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
    			if (firstBeacon->getPosition().second > secondBeacon->getPosition().second) {
        		theta_temp += calcAngle(firstBeacon->getPosition(), secondBeacon->getPosition());
        		theta_temp -= calcAngle(firstBeacon->getOffset(), secondBeacon->getOffset());
        		theta_temp += 90;
    			}
    			else {
        			theta_temp += calcAngle(secondBeacon->getPosition(), firstBeacon->getPosition());
        			theta_temp -= calcAngle(secondBeacon->getOffset(), firstBeacon->getOffset());
        			theta_temp += 90;
		}
		theta += theta_temp;
		++secondBeacon;
	}
	++firstBeacon;
	secondBeacon = firstBeacon;
	++secondBeacon;
}
theta /= (RoverBeacons.size() * numSieveBeacons);

/*rover_odometry::Odometry msg;
msg.x  = x;
msg.y  = y;
msg.th = th;
odometry_pub.publish(msg);*/

// std::cout << "x position: " << x << std::endl << "y position: " << y << std::endl << "Angle: " << theta << std::endl; 
}

//Always pass in the higher y-value as firstPos!!!!!!!
double Odometry::calcAngle(pair<double,double> firstPos, pair<double,double> secondPos) {
	double x_val = firstPos.first - secondPos.first;
	double y_val = firstPos.second - secondPos.second;
	//cout << "Calculating Angle: " << x_val << " " << y_val << endl;
	//cout << "The return value is: " << atan2(y_val,x_val) * (180/3.14159) << endl;
	return atan2(y_val,x_val) * (180/3.14159);
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
		getline(cin,line);//std::getline(driverData.out(),line);
		inputStream.str(line);
		getline(inputStream, beaconID, ',');
		getline(inputStream, sieveID, ',');
		getline(inputStream, distance_);
/*
		std::cout << "Line being read is: " << inputStream.str() << std::endl;
		std::cout << "Beacon IDs being read are: " << beaconID << std::endl;
		std::cout << "SieveID being read is: " << sieveID << std::endl;
		std::cout << "Distance being read is: " << distance_ << std::endl;
*/
		try {
			distance = stod(distance_);
			//std::cout << "addr " << RoverBeaconsMap[beaconID] << std::endl;
			RoverBeacon* temp = RoverBeaconsMap[beaconID];
			assert(temp);
			RoverBeaconsMap[beaconID]->updateReading(stoi(sieveID), distance);
			cout << "The distance between " << beaconID << " and " << sieveID << " is now " << RoverBeaconsMap[beaconID]->getReading(stoi(sieveID)) - RoverBeaconsMap[beaconID]->getBias(stoi(sieveID)) << std::endl;
			cout << RoverBeaconsMap[beaconID]->getBias(stoi(sieveID)) << endl;
			currentReadings.insert(beaconID + sieveID);
		}
		catch (std::invalid_argument error) {
			cerr<< "Invalid Argument: " << error.what() << "\n";
		}
	    
	}
	//std::cout << "Done with updating the readings" << std:: endl;	
}



pair<double,double> 
Odometry::getPosition(){
    return roverPosition;
}


double 
Odometry::getPose(){
    return roverPose;
}


