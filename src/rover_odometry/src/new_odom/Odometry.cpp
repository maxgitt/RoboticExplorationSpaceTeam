Odometry::Odometry(){
	std::string sieve_beacon_ids;
	std::string id;
	std::string offset;

	//Get list of sieve beacons addresses from parameter server
    nh.param("sieve_beacon_ids", sieve_beacon_ids, sieve_beacon_ids);
    //parse each address and check parameter server for offset. parameter server must be loaded
    //with xy offsets for each address of form x + <id>
    while(getline(sieve_beacon_ids, id, ",")){
    	std::string offsetparam = "x" + id;
    	nh.param(offsetparam, offset, offset);

    	std::string x;
    	std::string y;
    	getline(offset, x, ","));
    	getline(offset, y));

		std::pair<double,double> temp(stod(x),stod(y));
    	sieveBeacons.push_back(stoi(id),temp)
    }

    std::string prefix;
    std::string postfix;
    nh.param("driver_prefix", prefix, prefix);
    nh.param("driver_postfix", postfix, postfix);
    std::string driver_cmd = prefix + " " sieve_beacon_ids + " " postfix;

    driverData = popen(driver_cmd, "r");

    odom_pub     = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    current_time = ros::Time::now();
    last_time    = ros::Time::now();
}

void 
Odometry::detectBeacons(){
	int numSieveBeacons = sieveBeacons.size();
	std::set<std::string> uniqueRoverBeacons;

	std::cout << "Detecting the beacons that are connected to the rover\n";

    char buffer[256];
    std::string data;
    std::string id;
	while(!feof(rover_odometry.driverprogram_)){
    	if(fgets(buffer,256,rover_odometry.driverprogram_) != NULL){
    		data = buffer; //not sure if this is allowed
    		getline(data, id, ",");


    	}
    }
}


void 
Odometry::initializeBeacons(){
    for( auto rbeacon: roverBeacons){
        for( auto sbeacon: sieveBeacons){


        }
    }

    Beacon::Beacon temp();


}


void 
Odometry::calculateBiases(){



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



}














