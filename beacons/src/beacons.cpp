#include "beacons.h"

using std::string;
using std::ostream;using std::istream;
using std::cout;
using std::stringstream;

Beacon::Beacon(){}

Beacon::~Beacon(){}

ostream& operator<<(ostream& out, const Beacon& b){
	for( unsigned i = 0; i < b.values.size(); i++){
		out << b.values[i] << " ";
	}
	return out << "\n";
}

istream& operator>>(istream& in, Beacon& b){
	string token, sentence;
	stringstream iss;

	b.values.clear();

	getline(in, sentence);
	iss << sentence;
	while( getline(iss, token, ',') ){
		b.values.push_back( atof(token.c_str()) );
	}
	return in;
}



