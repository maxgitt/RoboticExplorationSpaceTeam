#include <Odometry/Odometry.h>
#include <sstream>
#include <string>
#include <stdio.h>
#include <set>
#include <vector>
#include <iostream>
#include <algorithm>
#include <time.h> //needed to run a while loop for a fixed amount of time

Odometry::Odometry(){ //default ctor
	std::string sieve_ids;
    std::string defaultVal =  "ERROR";
	std::string driverDefaultVal = DRIVER_PREFIX;
	std::string id; //individual sieve beacon
	std::string offsetValues; //x,y offset of the sieve beacon
    std::string beacon_ids;
    std::vector< std::pair<int, RoverBeacon::sieveBeaconData>> sieveBeacons;

	//Get list of sieve beacons addresses from parameter server
    nh.param("sieve_beacon_ids", sieve_ids, defaultVal);
    //parse each address and check parameter server for offset. parameter server must be loaded
    //with xy offsets for each address of form x + <id>
    std::istringstream sieve_beacon_ids(sieve_ids);
    while(getline(sieve_beacon_ids, id, ',')){
    	std::string offsetparam = "x" + id;
    	nh.param(offsetparam, offsetValues, defaultVal);
        std::istringstream offset(offsetValues);
    	std::string x = "-1";
    	std::string y = "-1";
    	getline(offset, x, ',');
    	getline(offset, y);

		std::pair<double,double> temp(stod(x),stod(y));
        RoverBeacon::sieveBeaconData newSieveBeaconData(temp);
        std::pair<int, RoverBeacon::sieveBeaconData> beaconToBeAdded(stoi(id),newSieveBeaconData);
        sieveBeacons.push_back(beaconToBeAdded);
    }
    
    //Initialize rover beacons
    nh.param("rover_beacon_ids", beacon_ids, defaultVal);
    std::istringstream rover_beacon_ids(beacon_ids);
    while (getline(rover_beacon_ids, id, ',')) {
        std::string offsetparam = "r" + id; //this is how rover offset is stored in the parameter server
        nh.param(offsetparam, offsetValues, defaultVal);
        std::istringstream offset(offsetValues);
        std::string x;
        std::string y;
        getline(offset, x, ',');
        getline(offset, y);
        
        std::pair<double,double> roverOffset(stod(x),stod(y));
        RoverBeacons.push_back(RoverBeacon(id, sieveBeacons, roverOffset));
    }
    //open a new process: the driver that reads the beacon data
    std::string prefix; //command that would run the
    nh.param("driver_prefix", prefix, driverDefaultVal);
    std::string driver_cmd = prefix + " " + sieve_beacon_ids.str() + DRIVER_POSTFIX;
    
    //popen opens a new process (essentially runs a command on the terminal)
    driverData = popen(driver_cmd.c_str(), "r"); //returns a pointer to an open stream and sets it up with read only permission
    //all driver data will be output to the driverData from this point on
    
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
    std::vector<std::string> uniqueRoverBeacons;
    std::cout << "Detecting the beacons that are connected to the rover\n";
    
    char buffer[256];
    std::string data;
    std::string currID;
    std::stringstream fileData;
    std::string roverBeaconID;
    bool timeout = false;
    time_t timeoutTime = time(NULL) + 30;
    //keep looking for the expected number of rover beacons until timeout at which point throw an exception
    while(!feof(driverData) and !timeout ){
        if (time(NULL) > timeoutTime) {
            timeout = true;
        }
        if(fgets(buffer, 256, driverData) != NULL){
            data = buffer; //not sure if this is allowed
            fileData.str(data); //put in a string stream
            getline(fileData, roverBeaconID, ',');
            if (std::find(uniqueRoverBeacons.begin(), uniqueRoverBeacons.end(), roverBeaconID) != uniqueRoverBeacons.end()) {
                uniqueRoverBeacons.push_back(roverBeaconID);
                
            }
            if (uniqueRoverBeacons.size() == numRoverBeacons) {
                return;
            }
        }
    }
    
    std::cout << "Expected " << numRoverBeacons << " beacons, but only " << uniqueRoverBeacons.size() << " unique rover beacons were detected!" << std::endl; //make sure handler excepts a boolean and it would indicate that rover beacons weren't all working
}

void 
Odometry::loadBiases(){
    std::string defaultValue = "ERROR";
    // In server, it should be designated "roverId sieveId" with value bias
    for (auto rbeacon: RoverBeacons) {
        std::unordered_map<int, RoverBeacon::sieveBeaconData> currBeaconReadings = rbeacon.getBeaconReadings();
        for (std::unordered_map<int, RoverBeacon::sieveBeaconData>::iterator it = currBeaconReadings.begin(); it != currBeaconReadings.end(); ++it) {
            std::string currBias;
            std::string currBiasPair = rbeacon.getId() + " " + std::to_string(it->first);
            nh.param(currBiasPair, currBias, defaultValue);
            rbeacon.updateBias(it->first, stod(currBias));
        }
    }
     


}


std::pair<double,double> 
Odometry::getPosition(){
    return roverPosition;
}


double 
Odometry::getPose(){
    return roverPose;
}


void 
Odometry::updateOdometry(){
    //update the previous values before calculating the new values
    x_prev = x;
    y_prev = y;
    th_prev = th;
    
    int x_val = 0;
    int y_val = 0;
    //take position readings from each rover beacon and then average them
    for (auto rbeacon: RoverBeacons) {
        x_val += (rbeacon.getPosition()).first + rbeacon.getOffset().first;
        y_val += (rbeacon.getPosition()).second + rbeacon.getOffset().second;
    }
    x = x_val/RoverBeacons.size();
    y = y_val/RoverBeacons.size();
    
    //calculate the new theta
    double theta = 0;
    int numPairs;
    std::vector<RoverBeacon>::iterator firstBeacon = RoverBeacons.begin();
    std::vector<RoverBeacon>::iterator secondBeacon = ++RoverBeacons.begin();
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
            ++numPairs;
        }
        
    }
    
    theta /= numPairs;
    
}

//Always pass in the higher y-value as firstPos!!!!!!!
double Odometry::calcAngle(std::pair<double,double> firstPos, std::pair<double,double> secondPos) {
    double x_val = firstPos.first - secondPos.first;
    double y_val = firstPos.second - secondPos.second;
    return atan2(y_val,x_val);
}
