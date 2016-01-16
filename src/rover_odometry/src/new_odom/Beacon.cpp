#include "beacon.h"

RoverBeacon::RoverBeacon(std::string id_) : id(id_){}

RoverBeacon::RoverBeacon(std::string id_, std::vector<std::pair<int, sieveBeaconData>> sieveBeacons, std::pair<double,double> roverOffset) : id(id_), offset(roverOffset) {
	for(auto beacon: sieveBeacons) //loop through the sieveBeacon vector to intialize the rover beacon;
		beaconReadings.insert({beacon.first, beacon.second}); //store the id of the sieve beacon and its associated data
}

double 
RoverBeacon::getReading(int id_){
	beaconIterator = beaconReadings.find(id_);
	return beaconIterator->second.reading;
}


void 
RoverBeacon::updateReading(int id_, double reading_){
	beaconIterator = beaconReadings.find(id_);
	beaconIterator->second.reading = reading_;
	positionUpdated = false;

}

double 
RoverBeacon::getBias(int id_){
	beaconIterator = beaconReadings.find(id_);
	return beaconIterator->second.bias;
}

void 
RoverBeacon::updateBias(int id_, double bias_){
	beaconIterator = beaconReadings.find(id_);
	beaconIterator->second.bias = bias_;
}

std::pair<double,double>
RoverBeacon::getPosition(){
	//Position of the Beacon should only be updated 
	//if new readings have been taken
	if(!positionUpdated) {
		updatePosition();
		positionUpdated = true;
	}
	return position;
}

std::unordered_map<int, RoverBeacon::sieveBeaconData> RoverBeacon::getBeaconReadings () {
    return beaconReadings;
}

void
RoverBeacon::updatePosition(){
	std::vector<std::pair<double,double>> positions; //vector of x,y coordinates based on all combinations of sieve beacons
	std::unordered_map<int,sieveBeaconData>::const_iterator firstBeacon  = beaconReadings.begin();
	if(firstBeacon == beaconReadings.end()) return; ///error must be thrown on need to include in RME that at least two beacons must be present
    std::unordered_map<int, sieveBeaconData>::const_iterator secondBeacon = ++beaconReadings.begin();
	double x;
	double y;
	while(secondBeacon != beaconReadings.end()){
		while(secondBeacon != beaconReadings.end()){
            std::pair<double,double> currPosition;
            /* We will use calculate the x and y position of the beacon based on the
             * intersection of the two circles with the radius equal to the distance b/t
             * a sieve beacon and a rover beacon. The offsets will be accounted for 
             * within the equation of the circle. These formulas can be found on:
             * http://www.ambrsoft.com/TrigoCalc/Circles2/Circle2.htm
             */
            // Circle one is of the form (x-a)^2 + (y-b)^2 = r0^2 where a is the x offset
            // of the sieve beacon and b is the y offset of the sieve beacon
            double r0 = firstBeacon->second.reading;
            double a = firstBeacon->second.offset.first;
            double b = firstBeacon->second.offset.second;
            
            // Circle two is of the form (x-c)^2 + (y-d)^2 = r1^2 where c is the x offset
            // of the sieve beacon and d is the y offset of the sieve beacon
            double r1 = secondBeacon->second.reading;
            double c = secondBeacon->second.offset.first;
            double d = secondBeacon->second.offset.second;
            
            //Calculate distance betwen circles
            double D = sqrt(pow((c-a),2) + pow((d-b),2));
            
            //Calculate the area of triangle formed by two circle centers
            double triArea = 0.25 * sqrt((D + r0 + r1) * (D + r0 - r1) * (D - r0 + r1) * (-D + r0 + r1));
            
            //There are two intersection points
            double x1 = (a+c)/2 + (((c - a) * (pow(r0,2) - pow(r1,2)))/(2 * pow(D,2))) + 2 * (b - d)/(pow(D,2)) * triArea;
            double x2 = (a+c)/2 + (((c - a) * (pow(r0,2) - pow(r1,2)))/(2 * pow(D,2))) - 2 * (b - d)/(pow(D,2)) * triArea;
            double y1 = (b+d)/2 + (((d - b) * (pow(r0,2) - pow(r1,2)))/(2 * pow(D,2))) - 2 * (a - c)/(pow(D,2)) * triArea;
            double y2 = (b+d)/2 + (((d - b) * (pow(r0,2) - pow(r1,2)))/(2 * pow(D,2))) + 2 * (a - c)/(pow(D,2)) * triArea;
            
            if (y1 < 0) {
                currPosition.first = x2;
                currPosition.second = y2;
            }
            else {
                currPosition.first = x1;
                currPosition.second = y1;
            }
            positions.push_back(currPosition);
            ++secondBeacon;
		}
		++firstBeacon;
		secondBeacon = ++firstBeacon;
	}
    //store the average x and y position of this beacon
    for (auto currPos : positions) {
        x += currPos.first;
        y += currPos.second;
    }
    
    position.first = x/positions.size();
    position.second = y/positions.size();

}

std::string
RoverBeacon::getId(){
	return id;
}

std::pair<double, double>
RoverBeacon::getOffset() {
    return offset;
}




