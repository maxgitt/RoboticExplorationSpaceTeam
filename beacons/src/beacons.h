#ifndef BEACONS_H
#define BEACONS_H
	
#include <unordered_map>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>

class Beacon{
	
	public:
		Beacon();
		Beacon(unsigned _id, pair<double, double> _offset, double _bias):
		id(_id), offset(_offset), bias(_bias){}
		

	private:	
		unsigned id;
		pair<double, double> offset;
		double bias;
		
};

class RoverBeacon : public Beacon {
	//offset is the offset from the center of the rover
}

class SieveBeacon : public Beacon {
	//offset is the offset from the left corner of the arena if you were standing behind the sieve
}

class BeaconEnv{
	
	BeaconEnv();

	friend std::ostream& operator<<(std::ostream& out, const BeaconEnv& b);
	friend std::istream& operator>>(std::istream& in, BeaconEnv& b);
	// <receiver id, sender id> ---------> <distance, last updated time>
	std::unordered_map< pair<unsigned, unsigned>, pair<double, long long>  > distances;
	SieveBeacon left;
	SieveBeacon right;
	RoverBeacon front;
	RoverBeacon back;
}

#endif
