/* 
 * File:   beacon.h
 * Author: pascualy
 *         Kishore B. Rao  Cell: 508-873-5384
 *         Bhairav Metha
 *
 * Description: a class that contain information regarding distance 
 * between one beacon on the rover and all sieve beacons
 *
 * Created on December 21st, 2015, 4:20 PM
 */

#ifndef BEACON_H
#define BEACON_H

#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <iostream>

using std::cout; 
using std::endl;
using std::pair; 
using std::vector;
using std::unordered_map; 
using std::string;

class RoverBeacon{
public:
	class sieveBeaconData{
		public:
		// xy pair which describes where the static beacon is with 
		// respect to the front center of the sieve
		pair<double,double> offset;
		double bias; //inherent bias of the decawave sensors
		double reading; //sensor reading
		sieveBeaconData(pair<double,double> offset_, 
			double bias_ = 0, 
				double reading_ = 0)
		: offset(offset_), bias(bias_), reading(reading_) {}
	};

	RoverBeacon(string id_);
	// Contructs a new Beacon object with id = id_ and sieve Beacons 
	// with their respective offset, bias and reading
	RoverBeacon(string id_, 
			vector<pair<int,sieveBeaconData>> sieveBeacons, 
				pair<double,double> roverOffset);
	
	double getReading(int id_);
	void updateReading(int id_, double reading_);
	
	double getBias(int id_);
	void updateBias(int id_, double bias_);
    
	pair<double, double> getOffset();

	pair<double,double> getPosition();
	void updatePosition();//REQUIRES: Need at least two sieve beacons

	string getId();
    
	unordered_map<int, sieveBeaconData> getBeaconReadings ();

private:
	string id; //unique id of the rove beacon
	pair<double,double> offset; //offset from center of the rover
	//position of beacon with respect to the center of the sieve, 
	pair<double,double> position; 
	bool positionUpdated = false;
	//same iterator share by an instance of this class
	unordered_map<int, sieveBeaconData>::iterator beaconIterator; 
	//the key of this map is the id of the beacon placed on the sieve.
	//the value is a sieveBeaconData object
	unordered_map<int, sieveBeaconData> beaconReadings;
};

#endif
