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
#include <deque>

class RoverBeacon{
public:
	class sieveBeaconData{
		public:
		// xy pair which describes where the static beacon is with 
		// respect to the front center of the sieve
		std::pair<double,double> offset;
		double bias; //inherent bias of the decawave sensors
		double reading; //sensor reading
		std::deque<double> all_readings;
		sieveBeaconData(std::pair<double,double> _offset, 
			double _bias = 0, 
				double _reading = 0)
		: offset(_offset), bias(_bias), reading(_reading) {}
	};

	RoverBeacon(std::string _id);
	// Contructs a new Beacon object with id = id_ and sieve Beacons 
	// with their respective offset, bias and reading
	RoverBeacon(std::string _id, 
		std::vector<std::pair<int,sieveBeaconData>> sieveBeacons, 
				std::pair<double,double> roverOffset);
	
	double getReading(int _id);
	void updateReading(int _id, double _reading);
	
	double getBias(int _id);
	void updateBias(int _id, double _bias);
    
	std::pair<double, double> getOffset();
	std::pair<double,double> getPosition();

	void updatePosition();//REQUIRES: Need at least two sieve beacons

	std::string getId();
	std::unordered_map<int, sieveBeaconData> getBeaconReadings();

private:
	std::string id; //unique id of the rove beacon
	std::pair<double,double> offset; //offset from center of the rover
	// position of beacon with respect to the center of the sieve, 
	std::pair<double,double> position; 
	bool positionUpdated = false;
	// same iterator share by an instance of this class
	std::unordered_map<int, sieveBeaconData>::iterator beaconIterator; 
	// the key of this map is the id of the beacon placed on the sieve.
	// the value is a sieveBeaconData object
	std::unordered_map<int, sieveBeaconData> beaconReadings;
  	// method for calculating position based on old position 
	// and new readings
  	std::pair<double, double> steepest_descent(
		const std::vector<double>& readings, 
		const std::vector<std::pair<double,double>>& offsets,
                std::pair<double, double>  old_pos);
    friend class Odometry;
  
};

#endif
