#include "beacons.h"
#include <string>
#include <stdlib.h>

Beacon::Beacon(){}

Beacon::~Beacon(){}

std::ostream& operator<<(std::ostream& out, const Beacon& b){
	for( unsigned i = 0; i < b.values.size(); i++){
		out << b.values[i] << " ";
	}
	return out << "\n";
}

std::istream& operator>>(std::istream& in, Beacon& b){
	std::string tmp;
	while( getline( in, tmp, ',' ) ){
		b.values.push_back(atof(tmp.c_str()));
	}
	return in;
}



