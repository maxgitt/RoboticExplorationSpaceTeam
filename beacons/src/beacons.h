#ifndef BEACONS_H
#define BEACONS_H
	

#include <unordered_map>
#include <vector>
#include <iostream>

class Beacon{
	
	public:
		Beacon();
		~Beacon();
		friend std::ostream& operator<<(std::ostream& out, const Beacon& b);
		friend std::istream& operator>>(std::istream& in, Beacon& b);
	private:	
		std::unordered_map<unsigned, double> distances;
		std::vector<double> values;
};


#endif
