#include <Odometry/Odometry.h>
#include <sstream>
#include <string>
#include <stdio.h>
#include <set>
#include <vector>
#include <iostream>
#include <algorithm>
#include <unordered_set>
#include <time.h> //needed to run a while loop for a fixed amount of time

Odometry::Odometry(){ //default ctor
	std::string sieve_ids;
	std::string driverDefaultVal = DRIVER_PREFIX;
	std::string id; //individual sieve beacon
	std::string offsetValues ="-100,-100"; //x,y offset of the sieve beacon
        std::string beacon_ids = "48c36259,35a79eb,997ca3f0,c3e3c227";
    	std::vector< std::pair<int, RoverBeacon::sieveBeaconData> > sieveBeacons;

    //Get list of sieve beacons addresses from parameter server
	
    nh.param(param_key + "sieve_beacon_ids", sieve_ids, sieve_ids);
    //parse each address and check parameter server for offset. parameter server must be loaded
    //with xy offsets for each address of form x + <id>
    std::istringstream sieve_beacon_ids(sieve_ids);
    while(getline(sieve_beacon_ids, id, ',')){
    	std::string offsetparam = "x" + id;
    	nh.param(param_key + offsetparam, offsetValues, offsetValues);
        std::istringstream offset(offsetValues);
    	std::string x = "-1";
    	std::string y = "-1";
    	getline(offset, x, ',');
    	getline(offset, y);

	std::pair<double,double> temp(stod(x),stod(y));
        RoverBeacon::sieveBeaconData newSieveBeaconData(temp);
        std::pair<int, RoverBeacon::sieveBeaconData> beaconToBeAdded(stoi(id),newSieveBeaconData);
        sieveBeacons.push_back(beaconToBeAdded);
        numSieveBeacons = sieveBeacons.size();
    }

    //Initialize rover beacons	
    nh.param(param_key + "rover_beacon_ids", beacon_ids, beacon_ids);
    std::istringstream rover_beacon_ids(beacon_ids);
    while (getline(rover_beacon_ids, id, ',')) {
        std::string offsetparam = param_key + "r" + id; //this is how rover offset is stored in the parameter server
        nh.param(offsetparam, offsetValues, offsetValues);
        std::istringstream offset(offsetValues);
        std::string x = "-1";
    	std::string y = "-1";
        getline(offset, x, ',');
        getline(offset, y);
        std::pair<double,double> roverOffset(stod(x),stod(y));
        RoverBeacons.push_back(RoverBeacon(id, sieveBeacons, roverOffset));
    }
    for(auto i : RoverBeacons){
        RoverBeaconsMap[i.getId()]= &i;
    }
    //open a new process: the driver that reads the beacon data
    std::string prefix; //command that would run the
    nh.param(param_key + "driver_prefix", prefix, driverDefaultVal);
    std::string driver_cmd = "sudo '/home/pascualy/catkin_ws/decawave_driver/dwm1000driver' 23,24";//prefix + " " + sieve_beacon_ids.str() + DRIVER_POSTFIX;
    
    //std::string driver_cmd = "script -c \"sudo '/home/pascualy/catkin_ws/decawave_driver/dwm1000driver' 13,24 \" /dev/null";
    //popen opens a new process (essentially runs a command on the terminal)
    driverData = popen(driver_cmd.c_str(), "r"); //returns a pointer to an open stream and sets it up with read only permission
if(setvbuf (driverData, NULL, _IOLBF, 0)) std::cout << "buffer not set";

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
    std::unordered_set<std::string> uniqueRoverBeacons;
    std::cout << "Detecting the beacons that are connected to the rover\n" << "We expect " << RoverBeacons.size() << " Beacons!" << std::endl;
    char buffer[128];
    std::string data;
    std::string currID;
    std::stringstream fileData;
    std::string roverBeaconID;
    time_t timeoutTime = time(NULL) + 10;
    //keep looking for the expected number of rover beacons until timeout at which point throw an exception
	if (!feof(driverData)) {
		fgets(buffer, sizeof buffer, driverData); //consume useless first line

	}
	 
    while(!feof(driverData)){
        /*if (time(NULL) > timeoutTime) {
	    std::cout << "Expected " << numRoverBeacons << " beacons, but only " << uniqueRoverBeacons.size() << " unique rover beacons were detected!" << std::endl; //make sure handler excepts a boolean and it would indicate that rover beacons weren't all working
            return;
        }*/
        if(fgets(buffer, sizeof buffer, driverData) != NULL){
            data = buffer; //not sure if this is allowed
	    //std::cerr << "Inisde the reading part, the data is: " << data << std::endl;
            fileData.str(data); //put in a string stream
            getline(fileData, roverBeaconID, ',');
	    std::cout << roverBeaconID << std::endl;
	    uniqueRoverBeacons.insert(roverBeaconID);
            if (uniqueRoverBeacons.size() == numRoverBeacons) {
		//std::cout << "About to close the pipe"<< std::endl;
		//pclose(driverData);
                return;
            }
        }
    }
    

}

void 
Odometry::loadBiases(){
    std::string defaultValue = "ERROR";
    // In server, it should be designated "roverId sieveId" with value bias
    for (auto rbeacon: RoverBeacons) {
        std::unordered_map<int, RoverBeacon::sieveBeaconData> currBeaconReadings = rbeacon.getBeaconReadings();
        for (std::unordered_map<int, RoverBeacon::sieveBeaconData>::iterator it = currBeaconReadings.begin(); it != currBeaconReadings.end(); ++it) {
            std::string currBias = "-1";
            std::string currBiasPair = rbeacon.getId() + "_" + std::to_string(it->first);
            nh.param(param_key + currBiasPair, currBias, defaultValue);
            rbeacon.updateBias(it->first, stod(currBias));
        }
    }
	
std::cout << "Reached end of load biases" << std::endl;
     


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
std::cout << "Reached update odometry!" << std::endl;
    updateBeaconReadings();
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
	   ++secondBeacon;
        }
        ++firstBeacon;
	secondBeacon = firstBeacon;
	++secondBeacon;
    }
    
    theta /= (RoverBeacons.size() * numSieveBeacons);
    
   std::cout << "x position: " << x << std::endl << "y position: " << std::endl << "Angle: " << theta << std::endl; 
}

//Always pass in the higher y-value as firstPos!!!!!!!
double Odometry::calcAngle(std::pair<double,double> firstPos, std::pair<double,double> secondPos) {
    double x_val = firstPos.first - secondPos.first;
    double y_val = firstPos.second - secondPos.second;
    return atan2(y_val,x_val);
}

//will store the values as a pair of id,reading for each beacon and update the readings. We will need to add a timeout!!
void Odometry::updateBeaconReadings() {
	std::string driverDefaultVal = DRIVER_PREFIX;
	std::string sieve_ids;
    std::unordered_set< std::string> currentReadings;
    std::string data;
    std::string beaconID;
    std::string sieveID;
    std::string distance_;
    double distance;
    std::stringstream inputStream;


	//open a new process: the driver that reads the beacon data

    //std::string driver_cmd = "script -c \"sudo '/home/pascualy/catkin_ws/decawave_driver/dwm1000driver' 13,24 \" /dev/null";
    //popen opens a new process (essentially runs a command on the terminal)
    //driverData = popen(driver_cmd.c_str(), "r"); //returns a pointer to an open stream and sets it up with read only permission
    	char buffer[128];
    	if(feof(driverData)) exit(1);
	std::cout << "About to get data from the file again" << std::endl;

	for(int i = 0; i < 100; ++i){
		fgets(buffer, sizeof buffer, driverData); //consume useless first line
		data = buffer; //convert to string
		inputStream.str(data);
		std::cout <<  inputStream.str() << std::endl;
	}
    //fflush(driverData);


    while (currentReadings.size() < (numSieveBeacons * RoverBeacons.size()) and !feof(driverData)) {
        
        if(fgets(buffer, sizeof buffer, driverData) != NULL) {
            data = buffer; //convert to string
            inputStream.str(data);
	    std::cout << "reading" << inputStream.str() << std::endl;
            getline(inputStream, beaconID, ',');
            getline(inputStream, sieveID, ',');
            getline(inputStream, distance_);
	    std::cout << "Distance: " << distance_ << std::endl;
	    //distance = stod(distance_);
            //RoverBeaconsMap[beaconID]->updateReading(stoi(sieveID), distance);
            //currentReadings.insert(beaconID + sieveID);
        }	
    }
}

















