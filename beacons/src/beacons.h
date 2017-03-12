#ifndef BEACONS_H
#define BEACONS_H
	
#include <unordered_map>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <utility>
#include <limits>
#include <cmath>

class Beacon{
	
	public:
		Beacon(){}
		Beacon(unsigned _id, std::pair<double, double> _offset, double _bias):
		id(_id), offset(_offset), bias(_bias){}
		
        double getReading();
        void setReading(double _reading);
        unsigned getID();
        std::pair<double, double> getOffset();
        double getBias(); 

		unsigned id;
        std::pair<double, double> offset;
		double bias;
        double reading = 0;
		
};

class sieveBeacon : public Beacon {
	//offset is the offset from the left corner of the arena if you were standing behind the sieve
};

class RoverBeacon : public Beacon {
	//offset is the offset from the center of the rover
    public:
	std::unordered_map< unsigned, sieveBeacon  > beaconReadings;
	std::unordered_map< unsigned, sieveBeacon  >::iterator beaconIterator;
    double getReading(int _id);
    void updateReading(int _id, double _reading);
    double getBias(int _id);
    void updateBias(int _id, double _bias);
    std::pair<double, double> steepest_descent(const std::vector<double>& readings, 
        const std::vector<std::pair<double,double>>& offsets,
        std::pair<double, double>  old_pos); 
};

class BeaconEnv{
    public:

	BeaconEnv(){}

    std::pair<double, double> position; 
    bool positionUpdated = false;
	friend std::ostream& operator<<(std::ostream& out, const BeaconEnv& b);
	friend std::istream& operator>>(std::istream& in, BeaconEnv& b);
	// <receiver id, sender id> ---------> <distance, last updated time>
	std::unordered_map< unsigned,RoverBeacon  > RoverBeacons;
	std::unordered_map< unsigned, RoverBeacon  >::iterator beaconIterator;
    
    std::pair<double, double> getPosition();

    void updatePosition();
    void publish();
    std::pair<double, double> steepest_descent(
        const std::vector<double>& readings, 
        const std::vector<std::pair<double,double>>& offsets,
        std::pair<double, double> old_pos); 
};

#endif
