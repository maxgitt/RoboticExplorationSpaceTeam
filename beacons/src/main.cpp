#include <iostream>
#include "beacons.h"

using std::cout;using std::cin;


int main(){
	Beacon test_beacon;
	cout << "Printing values: \n";
	while( cin >> test_beacon)
		cout << test_beacon;
	
}
