/* 
 * File:   beacon.h
 * Author: pascualy
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

class Beacon{
public:
	class beaconData{
		public:
		//xy pair which describes where the static beacon is with respect to the front center
		//of the sieve
		std::pair<double,double> offset;
		double bias;
		double reading;
		beaconData(std::pair<double,double> offset_, double bias_, double reading_): offset(offset_), bias(bias_), reading(reading_) {

		}
	};

	Beacon(std::string id_);
	//Contructs a new Beacon object with id = id_ and sieve Beacons with their respective
	//offset, bias and reading
	Beacon(std::string id_, std::vector<std::pair<int,beaconData>> sieveBeacons);
	
	double getReading(int id_);
	void updateReading(int id_, double reading_);
	
	double getBias(int id_);
	void updateBias(int id_, double bias_);

	std::pair<double,double> getPosition();
	void updatePosition();

	std::string getId();

private:
	std::string id;
	std::pair<double,double> offset;
	std::pair<double,double> position;
	bool positionUpdated = false;
	//the key of this map is the id of the beacon placed on the sieve.
	//the value is a pair containing with first equal to the distance reading 
	//and second equal to the bias of the beacon pair bias.
	std::unordered_map<int, beaconData>::iterator beaconIterator;
	std::unordered_map<int,beaconData> beaconReadings;
};

#endif