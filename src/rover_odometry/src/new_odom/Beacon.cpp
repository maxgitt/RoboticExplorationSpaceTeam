#include "beacon.h"

Beacon::Beacon(std::string id_) : id(id_){}

Beacon::Beacon(std::string id_, std::vector<std::pair<int,beaconData>> sieveBeacons) : id(id_) {
	for(auto beacon: sieveBeacons)
		beaconReadings.insert({beacon.first, beacon.second});
}

double 
Beacon::getReading(int id_){
	beaconIterator = beaconReadings.find(id_);
	return beaconIterator->second.reading;
}


void 
Beacon::updateReading(int id_, double reading_){
	beaconIterator = beaconReadings.find(id_);
	beaconIterator->second.reading = reading_;
	positionUpdated = false;

}

double 
Beacon::getBias(int id_){
	beaconIterator = beaconReadings.find(id_);
	return beaconIterator->second.bias;
}

void 
Beacon::updateBias(int id_, double bias_){
	beaconIterator 				  = beaconReadings.find(id_);
	beaconIterator->second.bias   = bias_;
}

std::pair<double,double>
Beacon::getPosition(){
	//Position of the Beacon should only be updated 
	//if new readings have been taken
	if(!positionUpdated) {
		updatePosition();
		positionUpdated = true;
	}
	return position;
}

void
Beacon::updatePosition(){
	std::vector<std::pair<double,double>> positions;
	std::unordered_map<int,beaconData>::const_iterator first  = beaconReadings.begin();
	if(first == beaconReadings.end()) return; ///error must be thrown on need to include in RME that at least two beacons must be present
	std::unordered_map<int,beaconData>::const_iterator second = beaconReadings.begin() + 1;
	double x;
	double y;
	while(second != beaconReadings.end()){
		while(second != beaconReadings.end()){

		}
		++first;
		second = first + 1;
	}

	//equation must be genrealized for sieve beacons that are in any xy position.
    //x = pow(sieve_offset_left_,2) - pow(sieve_offset_left_,2) + pow(rover_fwd_sieve_right_, 2) - pow(rover_fwd_sieve_left_, 2);
    //x /= (-2*(-sieve_offset_left_ + sieve_offset_right_));
    //
    //y = sqrt(abs(pow(rover_fwd_sieve_left_, 2) - pow(x_fwd - sieve_offset_left_, 2)));
}


std::string 
Beacon::getId(){
	return id;
}



