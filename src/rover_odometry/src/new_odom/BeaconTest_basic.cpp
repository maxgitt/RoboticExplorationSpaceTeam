#include "Beacon.h"
#include <cassert>

int main(){
	std::string id = "2feo290";

	//test basic constructor
	Beacon * test = new Beacon(id);
	assert(test->getId() == id);
	delete test;
	test = NULL;
	
	std::vector<std::pair<int,Beacon::beaconData>> sieveBeacons;

	std::pair<double,double> testOffset(-1,0);
	std::pair<int, Beacon::beaconData> testBeacon(1, Beacon::beaconData(testOffset, 10, 10));
	sieveBeacons.push_back(testBeacon);

	test = new Beacon(id, sieveBeacons);
	assert(test->getBias(1) == 10);
	assert(test->getReading(1) == 10);

	test->updateBias(1,30);
	test->updateReading(1, 20);

	assert(test->getBias(1) == 30);
	assert(test->getReading(1) == 20);
	return 0;
}