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
	string sieve_ids = "13,24,23"; //default sieve IDs
	string id; //individual sieve beacon
	string offsetValues ="-100,-100"; //default x,y offset of the sieve beacon
	string beacon_ids = "48c36259,35a79eb,997ca3f0,c3e3c227"; //default rover IDs
	vector< std::pair<int, RoverBeacon::sieveBeaconData> > sieveBeacons;

	//Get list of sieve beacons addresses from parameter server
	nh.param(param_key + "sieve_beacon_ids", sieve_ids, sieve_ids);
	/* Parse each address and check parameter server for offset.
     * parameter server must be loaded
     * with xy offsets for each address of form x + <id> */
    
#if DEBUG
	cerr << "Read the following sieve_ids from the parameter server: " <<
							 sieve_ids << endl;
#endif
    
	istringstream sieve_beacon_ids(sieve_ids);
	while(getline(sieve_beacon_ids, id, ',')){
		string offsetparam = "x" + id;
		nh.param(param_key + offsetparam, offsetValues, offsetValues);
        
#if DEBUG
		cerr << "The offset for sieve_id: " << id <<
					" is " << offsetValues << endl;
#endif
        
        //Obtain x and y offsets for all the sieve becaons
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
    
#if DEBUG
	cerr << "Loading the following rover beacon ids from the parameter server: " << beacon_ids << endl;
#endif
    
	istringstream rover_beacon_ids(beacon_ids);
	while (getline(rover_beacon_ids, id, ',')) {
		//this is how rover offset is stored in the parameter server
		string offsetparam = param_key + "r" + id;
		nh.param(offsetparam, offsetValues, offsetValues);
        
#if DEBUG
		cerr << "Rover beacon: " << id << " has offsets " <<
							 offsetValues;
#endif
        
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
#if DEBUG
	cerr << "Detecting the beacons that are connected to the rover\n"
		<< "We expect " << RoverBeacons.size() << " Beacons!\n"
    << "We also expect " << numSieveBeacons << " sieve beacons." << endl;
    
#endif 
    
    unordered_set<std::string> currentReadings;
    string beaconID, sieveID, distance_;
    getline(cin,beaocnID); //consume useless first line
    timer.recordTime();
    double startTime = timer.getTime();
    double time_difference = 0;
    while (currentReadings.size() < (numSieveBeacons * RoverBeacons.size()) && time_difference < 30 ) {
        stringstream inputStream;
        string line;
        getline(cin,line);
        
#if FULLDEBUG
        cerr << "Current Line being read: " << line << endl;
        
#endif
        
        inputStream.str(line);
        getline(inputStream, beaconID, ',');
        getline(inputStream, sieveID, ',');
        getline(inputStream, distance_);
        currentReadings.insert(beaconID + sieveID);
        timer.recordTime();
        time_difference = timer.getTime() - startTime;
    }
    
    if (time_difference >= 30) {
        cerr << "Exiting, could not get readings from all beacons in a 30 second interval\n";
        exit(1);
    }
    
    cout << "Detected all " << numSieveBeacons * RoverBeacons.size() << " combinations"
    << " of rover and sieve beacons!\n";

}

void 
Odometry::loadBiases(){
    cout << "Please place the rover (center of the rover) at position (0,4) with the center of the sieve being (0,0)\n";
    timer.recordTime();
    double startTime = timer.getTime();
    time_difference = 0;
    
    while (time_difference < 480) { //wait for 8 minutes
        timer.recordTime();
        time_difference = timer.getTime() - startTIme;
    }
    
    auto roverBeaconIt = roverBeacons.begin();
    auto endRoverBeaconIt = roverBeacons.end();
    for (; roverBeaconIt != endRoverBeaconIt; ++roverBeaconIt) {
        double rBeaconXPos = roverBeaconIt->getOffset().first;
        double rBeaconYPos = roverBeaconIt->getOffset().second + 4;
        unordered_map<int, sieveBeaconData>::iterator sieveIt = roverBeaconIt->beaconReadings.begin();
        unordered_map<int, sieveBeaconData>::iterator endSieveIt = roverBeaconIt->beaconReadings.end();
        for (; sieveIt != endSieveIt; ++sieveIt) {
            double sieveXPos = sieveIt->second.offset.first();
            double sieveYPos = sieveIt->second.offset.second();
            double expectedValue = sqrt(pow((rBeaconXPos - sieveXPos),2) + pow((rBeaconYPos - sieveBeaconYPos), 2));
            int currSieveID = sieveIt->first;
            bool found = false;
            while (!found) {
                stringstream inputStream;
                string line;
                getline(cin,line);
                string beaconID, sieveID, distance_;
#if FULLDEBUG
                cerr << "Current Line being read: " << line << endl;
                
#endif
                inputStream.str(line);
                getline(inputStream, beaconID, ',');
                getline(inputStream, sieveID, ',');
                getline(inputStream, distance_);
                if (beaconID == roverBeaconIt->id) {
                    if (stoi(sieveID) == currSieveID) {
                        found = true;
                        double actualValue = stod(distance_);
                        double bias = actualValue - expectedValue;
#if DEBUG
                        cerr << "Loading a bias of " << bias << " between rover beacon " << beaconID
                        << " and sieve beacon " << sieveID << " because the real distance is "
                        << expectedValue << " and the sensors produce a value of " << actualValue << endl;
#endif
                        
                        roverBeaconIt->updateBias(currSieveID, bias);
                    }
                }

            }
    }
        
        
    cout << "All rover beacons have been fully configured. We are ready to output beautiful odometry data!\n";
    
#if DEBUG
        cout << "Here is all of the beacon data\n";
        roverBeaconIt = roverBeacons.begin();
        endRoverBeaconIt = roverBeacons.end();
        for (; roverBeaconIt != endRoverBeaconIt; ++roverBeaconIt) {
            sieveIt = roverBeaconIt->beaconReadings.begin();
            endSieveIt = roverBeaconIt->beaconReadings.end();
            cout << roverBeaconIt->id << " has an offset of " << roverBeaconIt->offset.first << ","
            << roverBeaconIt->offset.second << " and the following associated sieve beacons\n";
            for (; sieveIt != endSieveIt; ++sieveIt) {
                cout << "     " << sieveIt->first << " has an offset of " << sieveIt->offset.first << ","
                << sieveIt->offset.second << " and a bias of " << sieveIt->bias << "\n";
            }
        }
#endif

}


void 
Odometry::updateOdometry(){

#if FULLDEBUG
	cout << "Reached update odometry!" << endl;
#endif 
    
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
		x_val += ((*it).getPosition()).first - (*it).getOffset().first;
		y_val += ((*it).getPosition()).second - (*it).getOffset().second;
#if DEBUG
        cerr << it->id << " predicts that the center of the rover is at " << x_val "," << y_val << "\n";
#endif
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
			theta += theta_temp;
			++secondBeacon;
		}
		++firstBeacon;
		secondBeacon = firstBeacon;
		++secondBeacon;
	}
	//change when we add more rover beacons
	theta /= 1;
    
#if DEBUG
    cout << endl << endl;
	std::cout << "x position: " << x << std::endl << "y position: " << y << std::endl << "Angle: " << theta << std::endl; 
    cout << endl << endl;
#endif
    
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
	return return_angle;
}

//will store the values as a pair of id,reading for each beacon and update the readings. We will need to add a timeout!!
void Odometry::updateBeaconReadings() {
    
#if FULLDEBUG
	cout << "Going to update the Beacon Readings\n";
#endif
    
    
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
#if FULLDEBUG
        cout << "Current line of data being read is: " << line << endl;
#endif
        
        
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


