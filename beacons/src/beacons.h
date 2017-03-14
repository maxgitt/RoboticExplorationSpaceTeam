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
    public:
    sieveBeacon(){}
    sieveBeacon(unsigned _id, std::pair<double, double> _offset, double _bias):
		Beacon(_id,_offset, _bias){}
};

class RoverBeacon : public Beacon {
	//offset is the offset from the center of the rover
    public:
    RoverBeacon(){}
    RoverBeacon(unsigned _id, std::pair<double, double> _offset, double _bias):
        Beacon(_id,_offset, _bias){
            beaconReadings[50] = sieveBeacon(50, std::pair<double, double>(1,0),0);   
        }
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

	BeaconEnv(){
        RoverBeacons[64] = RoverBeacon(64,  std::pair<double, double>(.2,0),0);
        RoverBeacons[5] = RoverBeacon(5, std::pair<double, double>(-.2,0),0);
    }
    std::pair<double, double> position= std::pair<double, double>(3,0); 
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
